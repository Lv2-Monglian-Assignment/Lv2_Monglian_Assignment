"""Compare live Image/PointStamped headers and target publisher QoS.

This subscribes only. It does not launch or stop the camera/detector, and is
an interface check rather than a labeled detection accuracy evaluation.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time


def header_dict(msg):
    return {"stamp": {"sec": msg.header.stamp.sec,
                      "nanosec": msg.header.stamp.nanosec},
            "frame_id": msg.header.frame_id}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--target-topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--matches", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--expect-zero", action="store_true")
    args = parser.parse_args()
    if (not args.image_topic or not args.target_topic or args.matches < 1
            or not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("토픽은 비어 있을 수 없으며 matches와 timeout은 양수여야 합니다.")
    out = Path(args.output).expanduser().resolve()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"기존 결과를 보존합니다. 새 --output을 지정하세요: {out}")
        return 2
    source = Path(__file__).read_bytes()
    (out / "check_source.py").write_bytes(source)
    result = {
        "purpose": "Live header/QoS interface check; not labeled accuracy evaluation",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "image_topic": args.image_topic, "target_topic": args.target_topic,
        "requested_pairs": args.matches, "timeout_seconds": args.timeout,
        "expect_zero": args.expect_zero,
        "environment": {k: os.environ.get(k) for k in (
            "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "ROS_STATIC_PEERS", "CYCLONEDDS_URI")},
        "source_sha256": hashlib.sha256(source).hexdigest(),
        "image_messages": 0, "target_messages": 0,
        "pending_entries_evicted": 0, "pairs": [],
        "header_mismatches": 0, "nonzero_or_invalid_pairs": 0,
        "target_publishers": [], "target_qos_ok": False,
        "passed": False, "error": None,
    }
    ros = node = None
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import PointStamped

        ros = rclpy
        ros.init()
        node = ros.create_node("vision_target_header_check_" + str(os.getpid()))
        images, targets, paired = {}, {}, set()

        def receive(msg, is_image):
            counter = "image_messages" if is_image else "target_messages"
            result[counter] += 1
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if key in paired or len(result["pairs"]) >= args.matches:
                return
            entry = {"header": header_dict(msg)}
            if is_image:
                entry.update(width=msg.width, height=msg.height, encoding=msg.encoding)
            else:
                entry["point"] = [msg.point.x, msg.point.y, msg.point.z]
            pending = images if is_image else targets
            pending[key] = entry
            if key in images and key in targets:
                image, target = images.pop(key), targets.pop(key)
                header_ok = image["header"] == target["header"]
                zero_ok = all(math.isfinite(v) and v == 0 for v in target["point"])
                result["pairs"].append({
                    "input": image, "output": target,
                    "header_equal": header_ok, "point_is_zero": zero_ok,
                })
                paired.add(key)
                result["header_mismatches"] += int(not header_ok)
                result["nonzero_or_invalid_pairs"] += int(not zero_ok)
            # Keep metadata only and bound unmatched entries; no image pixels saved.
            for cache in (images, targets):
                while len(cache) > max(256, args.matches):
                    cache.pop(next(iter(cache)))
                    result["pending_entries_evicted"] += 1

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(Image, args.image_topic, lambda m: receive(m, True), qos)
        node.create_subscription(PointStamped, args.target_topic,
                                 lambda m: receive(m, False), qos)
        print(f"입력·출력 header {args.matches}쌍 대조 대기 (최대 {args.timeout:g}초)", flush=True)
        deadline = time.monotonic() + args.timeout
        while len(result["pairs"]) < args.matches and time.monotonic() < deadline:
            ros.spin_once(node, timeout_sec=min(.1, max(0., deadline - time.monotonic())))
        publishers = node.get_publishers_info_by_topic(args.target_topic)
        for info in publishers:
            q = info.qos_profile
            result["target_publishers"].append({
                "node_name": info.node_name, "node_namespace": info.node_namespace,
                "reliability": q.reliability.name, "history": q.history.name,
                "depth": q.depth, "durability": q.durability.name,
            })
        result["target_qos_ok"] = (
            len(publishers) == 1 and publishers[0].node_name == "target_detector"
            and publishers[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            and publishers[0].qos_profile.history == HistoryPolicy.KEEP_LAST
            and publishers[0].qos_profile.depth == 1)
        result["unmatched_image_headers"] = len(images)
        result["unmatched_target_headers"] = len(targets)
        if len(result["pairs"]) < args.matches:
            result["error"] = "제한시간 내 같은 stamp의 입력·출력 쌍을 충분히 받지 못했습니다."
        elif result["header_mismatches"]:
            result["error"] = "같은 stamp의 입력·출력 frame_id가 다릅니다."
        elif args.expect_zero and result["nonzero_or_invalid_pairs"]:
            result["error"] = "대조한 출력 중 0이 아닌 값 또는 유효하지 않은 값이 있습니다."
        elif not result["target_qos_ok"]:
            result["error"] = "단일 target_detector 발행자의 Best-effort / Keep-last 1을 확인하지 못했습니다."
        else:
            result["passed"] = True
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = str(exc) or "Interrupted"
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if ros is not None and ros.ok():
                ros.shutdown()
        except Exception as exc:
            result["cleanup_error"] = str(exc)
        (out / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"matched_pairs": len(result["pairs"]),
                      "header_mismatches": result["header_mismatches"],
                      "nonzero_or_invalid_pairs": result["nonzero_or_invalid_pairs"],
                      "target_qos_ok": result["target_qos_ok"],
                      "passed": result["passed"], "error": result["error"]},
                     ensure_ascii=False), flush=True)
    print("결과 폴더:", out, flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
