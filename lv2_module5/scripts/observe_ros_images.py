"""Measure local ROS Image arrivals without OpenCV, pixel saving or device control."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import socket
import time
import math


def kernel_counters():
    wanted = {"Ip": {"ReasmFails", "ReasmTimeout"}, "Udp": {"InErrors", "RcvbufErrors"}}
    try:
        lines = Path("/proc/net/snmp").read_text().splitlines()
        counters = {}
        for i in range(0, len(lines) - 1, 2):
            names, values = lines[i].split(), lines[i + 1].split()
            group = names[0].rstrip(":")
            if names[0] != values[0] or len(names) != len(values):
                continue
            for key, value in zip(names[1:], values[1:]):
                if key in wanted.get(group, set()):
                    counters[group + "." + key] = int(value)
        return counters
    except (OSError, ValueError) as exc:
        return {"read_error": str(exc)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not args.topic or not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error("토픽과 0초 초과·60초 이하 관측 시간을 지정하세요.")
    out = Path(args.output).expanduser().resolve()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"기존 결과를 보존합니다. 새 --output을 지정하세요: {out}")
        return 2
    source = Path(__file__).read_bytes()
    (out / "observe_source.py").write_bytes(source)
    result = {
        "purpose": "Local ROS Image arrival measurement; not physical camera FPS or accuracy evaluation",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": socket.gethostname(), "topic": args.topic,
        "seconds_requested": args.seconds,
        "subscription_qos": "best_effort / keep_last depth 1 / volatile",
        "environment": {key: os.environ.get(key) for key in (
            "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "ROS_STATIC_PEERS", "CYCLONEDDS_URI")},
        "source_sha256": hashlib.sha256(source).hexdigest(),
        "samples": [], "status": "not_finished", "error": None,
    }
    ros = node = None
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        ros = rclpy
        ros.init()
        node = ros.create_node("vision_image_stream_observer_" + str(os.getpid()))
        result["kernel_counters_before"] = kernel_counters()
        started = time.monotonic()

        def receive(msg):
            result["samples"].append({
                "arrival_seconds": time.monotonic() - started,
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id, "encoding": msg.encoding,
                "width": msg.width, "height": msg.height, "step": msg.step,
                "payload_bytes": len(msg.data),
            })

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        subscription = node.create_subscription(Image, args.topic, receive, qos)
        print(f"HOST={result['host']}: ROS 컬러 입력을 {args.seconds:g}초 관측합니다.", flush=True)
        while ros.ok() and time.monotonic() - started < args.seconds:
            ros.spin_once(node, timeout_sec=min(0.1, max(0, args.seconds - (time.monotonic() - started))))
        result["elapsed_seconds"] = time.monotonic() - started
        result["status"] = "complete" if result["elapsed_seconds"] >= args.seconds else "stopped_early"
    except (Exception, KeyboardInterrupt) as exc:
        result["status"] = "error_or_interrupted"
        result["error"] = str(exc) or "Interrupted"
    finally:
        result["kernel_counters_after"] = kernel_counters()
        before, after = result.get("kernel_counters_before", {}), result["kernel_counters_after"]
        result["kernel_counters_delta"] = {key: after[key] - value for key, value in before.items()
                                           if isinstance(value, int) and isinstance(after.get(key), int)}
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                result["cleanup_error"] = str(exc)
        if ros is not None and ros.ok():
            try:
                ros.shutdown()
            except Exception as exc:
                result["cleanup_error"] = str(exc)
        samples = result["samples"]
        arrival_gaps = [b["arrival_seconds"] - a["arrival_seconds"] for a, b in zip(samples, samples[1:])]
        stamps = [s["stamp"]["sec"] * 1_000_000_000 + s["stamp"]["nanosec"] for s in samples]
        stamp_gaps = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
        span = samples[-1]["arrival_seconds"] - samples[0]["arrival_seconds"] if len(samples) > 1 else 0
        source_span = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0
        result.update({
            "image_messages": len(samples),
            "arrival_hz_between_first_and_last": (len(samples) - 1) / span if span > 0 else None,
            "observed_header_rate_hz": (len(samples) - 1) / source_span if source_span > 0 else None,
            "max_arrival_gap_seconds": max(arrival_gaps, default=None),
            "max_observed_stamp_gap_seconds": max(stamp_gaps, default=None),
            "source_stamps_strictly_increasing": bool(stamp_gaps) and all(g > 0 for g in stamp_gaps),
            "input_formats": [list(v) for v in sorted({(s["width"], s["height"], s["encoding"]) for s in samples})],
        })
        (out / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    keys = ("host", "image_messages", "arrival_hz_between_first_and_last", "observed_header_rate_hz",
            "max_arrival_gap_seconds", "max_observed_stamp_gap_seconds", "input_formats",
            "kernel_counters_delta", "status", "error")
    print(json.dumps({key: result.get(key) for key in keys}, ensure_ascii=False), flush=True)
    print("결과 폴더:", out, flush=True)
    return 0 if result["status"] == "complete" and result["image_messages"] > 0 and "cleanup_error" not in result else 1


if __name__ == "__main__":
    raise SystemExit(main())
