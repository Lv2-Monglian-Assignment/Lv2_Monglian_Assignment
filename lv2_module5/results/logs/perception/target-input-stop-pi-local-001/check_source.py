"""Observe a short PointStamped stream; no Image subscription or device control."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time


def finite_json(value):
    """Keep malformed non-finite point values recordable in standard JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_json(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--min-messages", type=int, default=30)
    parser.add_argument("--max-gap", type=float, default=0.5)
    parser.add_argument("--frame-id", default="camera_color_optical_frame")
    args = parser.parse_args()
    if (not args.topic or not args.frame_id or args.min_messages < 2
            or not math.isfinite(args.seconds) or not 0 < args.seconds <= 60
            or not math.isfinite(args.max_gap) or args.max_gap <= 0):
        parser.error("토픽·frame_id, 2개 이상 메시지, 0~60초 관측 및 양수 간격을 지정하세요.")
    out = Path(args.output).expanduser().resolve()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"기존 결과를 보존합니다. 새 --output을 지정하세요: {out}")
        return 2
    source = Path(__file__).read_bytes()
    (out / "check_source.py").write_bytes(source)
    result = {
        "purpose": "Short target delivery observation; not accuracy or long-term stability evaluation",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "topic": args.topic, "seconds_requested": args.seconds,
        "criteria": {"min_messages": args.min_messages, "max_gap_seconds": args.max_gap,
                     "frame_id": args.frame_id},
        "subscription_qos": "best_effort / keep_last depth 1 / volatile",
        "environment": {key: os.environ.get(key) for key in (
            "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "ROS_STATIC_PEERS", "CYCLONEDDS_URI")},
        "source_sha256": hashlib.sha256(source).hexdigest(),
        "samples": [], "publisher_qos_ok": False, "passed": False, "errors": [],
    }
    ros = node = None
    try:
        import rclpy
        from geometry_msgs.msg import PointStamped
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        ros = rclpy
        ros.init()
        node = ros.create_node("vision_target_stream_check_" + str(os.getpid()))
        started = time.monotonic()

        def receive(msg):
            result["samples"].append({
                "arrival_seconds": time.monotonic() - started,
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
                "point": [msg.point.x, msg.point.y, msg.point.z],
            })

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        subscription = node.create_subscription(PointStamped, args.topic, receive, qos)
        print(f"/target 작은 메시지만 {args.seconds:g}초 관측합니다. 영상은 추가 구독하지 않습니다.", flush=True)
        while ros.ok() and time.monotonic() - started < args.seconds:
            ros.spin_once(node, timeout_sec=min(0.1, max(0, args.seconds - (time.monotonic() - started))))
        elapsed = time.monotonic() - started
        result["elapsed_seconds"] = elapsed
        pubs = node.get_publishers_info_by_topic(args.topic)
        result["publishers"] = [{"node_name": p.node_name, "node_namespace": p.node_namespace,
                                 "reliability": p.qos_profile.reliability.name,
                                 "history": p.qos_profile.history.name, "depth": p.qos_profile.depth,
                                 "durability": p.qos_profile.durability.name} for p in pubs]
        result["publisher_qos_ok"] = (
            len(pubs) == 1 and pubs[0].node_name == "target_detector"
            and pubs[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            and pubs[0].qos_profile.history == HistoryPolicy.KEEP_LAST
            and pubs[0].qos_profile.depth == 1
            and pubs[0].qos_profile.durability == DurabilityPolicy.VOLATILE)
        samples = result["samples"]
        result["messages_received"] = len(samples)
        invalid = 0
        stamps = []
        for sample in samples:
            sec, ns = sample["stamp"]["sec"], sample["stamp"]["nanosec"]
            x, y, z = sample["point"]
            stamps.append(sec * 1_000_000_000 + ns)
            valid = (sec > 0 and 0 <= ns < 1_000_000_000
                     and sample["frame_id"] == args.frame_id
                     and all(math.isfinite(v) for v in (x, y, z))
                     and -1 <= x <= 1 and -1 <= y <= 1 and 0 <= z <= 1
                     and (z != 0 or x == y == 0))
            invalid += not valid
        arrival_gaps = [b["arrival_seconds"] - a["arrival_seconds"] for a, b in zip(samples, samples[1:])]
        stamp_gaps = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
        tail = elapsed - samples[-1]["arrival_seconds"] if samples else elapsed
        result.update({
            "invalid_messages": invalid,
            "source_stamps_strictly_increasing": bool(stamp_gaps) and all(g > 0 for g in stamp_gaps),
            "max_arrival_gap_seconds": max(arrival_gaps, default=None),
            "max_observed_stamp_gap_seconds": max(stamp_gaps, default=None),
            "silence_after_last_message_seconds": tail,
            "arrival_hz_between_first_and_last": ((len(samples) - 1) / (samples[-1]["arrival_seconds"] - samples[0]["arrival_seconds"])) if len(samples) > 1 else None,
        })
        if len(samples) < args.min_messages:
            result["errors"].append("관측 메시지 수가 지정한 최소 기준보다 적습니다.")
        if invalid:
            result["errors"].append("header 또는 point가 유효성 기준을 벗어났습니다.")
        if not result["source_stamps_strictly_increasing"]:
            result["errors"].append("관측 stamp가 단조 증가하지 않거나 대조할 메시지가 부족합니다.")
        if max(arrival_gaps + stamp_gaps + [tail]) > args.max_gap:
            result["errors"].append("관측 간격 또는 마지막 메시지 이후 무수신 시간이 지정한 상한을 넘었습니다.")
        if not result["publisher_qos_ok"]:
            result["errors"].append("단일 target_detector 발행자의 요구 QoS를 확인하지 못했습니다.")
        result["passed"] = not result["errors"]
    except (Exception, KeyboardInterrupt) as exc:
        result["errors"].append(str(exc) or "Interrupted")
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if ros is not None and ros.ok():
                ros.shutdown()
        except Exception as exc:
            result["cleanup_error"] = str(exc)
            result["passed"] = False
        (out / "result.json").write_text(json.dumps(finite_json(result), ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    summary_keys = ("messages_received", "arrival_hz_between_first_and_last", "max_arrival_gap_seconds",
                    "max_observed_stamp_gap_seconds", "source_stamps_strictly_increasing",
                    "invalid_messages", "publisher_qos_ok", "passed", "errors")
    print(json.dumps(finite_json({key: result.get(key) for key in summary_keys}), ensure_ascii=False, allow_nan=False), flush=True)
    print("결과 폴더:", out, flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
