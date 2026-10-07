"""Save distinct real ROS color frames for a physically labelled evaluation scene."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene", choices=("present", "absent", "unlabelled"), required=True)
    parser.add_argument("--label-basis", required=True, help="Independent physical scene confirmation")
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    if (not args.topic or not args.label_basis.strip() or not 1 <= args.count <= 100
            or not math.isfinite(args.interval) or args.interval < 0
            or not math.isfinite(args.timeout) or not 0 < args.timeout <= 300):
        parser.error("Specify a topic, physical label basis, 1..100 frames and finite time limits.")
    out = Path(args.output).expanduser().resolve()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f"Existing dataset preserved; choose a new --output: {out}")
    source = Path(__file__).read_bytes()
    (out / "capture_source.py").write_bytes(source)
    result = {
        "purpose": "Real camera evaluation frames; labels describe physical scene setup",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": socket.gethostname(), "topic": args.topic,
        "scene": args.scene, "label_basis": args.label_basis,
        "frames_requested": args.count, "interval_seconds": args.interval,
        "timeout_seconds": args.timeout, "source_sha256": hashlib.sha256(source).hexdigest(),
        "environment": {k: os.environ.get(k) for k in (
            "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI")},
        "subscription_qos": "best_effort / keep_last depth 1 / volatile",
        "camera_launched_by_this_tool": False,
        "frames": [], "images_received": 0, "error": None, "success": False,
    }
    ros = node = None
    started = time.monotonic()
    try:
        import cv2
        from cv_bridge import CvBridge
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        result["opencv_version"] = cv2.__version__
        ros = rclpy
        ros.init()
        node = ros.create_node("vision_evaluation_capture_" + str(os.getpid()))
        bridge = CvBridge()
        last_saved = -math.inf
        last_stamp = -1

        def receive(msg):
            nonlocal last_saved, last_stamp
            result["images_received"] += 1
            now = time.monotonic()
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if len(result["frames"]) >= args.count or now - last_saved < args.interval:
                return
            if stamp <= last_stamp:
                return
            if (msg.encoding not in ("rgb8", "bgr8") or msg.width <= 0 or msg.height <= 0
                    or msg.header.stamp.sec <= 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000):
                raise ValueError("Invalid color Image format or source stamp")
            bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if bgr.shape != (msg.height, msg.width, 3) or bgr.dtype.name != "uint8":
                raise ValueError("Invalid converted color array")
            filename = f"frame-{len(result['frames']) + 1:03d}.png"
            path = out / filename
            if not cv2.imwrite(str(path), bgr):
                raise RuntimeError("PNG write failed")
            result["frames"].append({
                "file": filename, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "target_present": None if args.scene == "unlabelled" else args.scene == "present",
                "arrival_seconds": now - started,
                "header": {"stamp": {"sec": msg.header.stamp.sec,
                                     "nanosec": msg.header.stamp.nanosec},
                           "frame_id": msg.header.frame_id},
                "encoding": msg.encoding, "width": msg.width, "height": msg.height,
                "file_encoding": "BGR passed to OpenCV imwrite",
            })
            last_saved, last_stamp = now, stamp

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        subscription = node.create_subscription(Image, args.topic, receive, qos)
        started = time.monotonic()
        print(f"CAPTURE_START: host={result['host']} scene={args.scene} frames={args.count}", flush=True)
        while ros.ok() and len(result["frames"]) < args.count and time.monotonic() - started < args.timeout:
            ros.spin_once(node, timeout_sec=0.1)
        result["success"] = len(result["frames"]) == args.count
        if not result["success"]:
            result["error"] = "Not enough distinct real Image frames before timeout"
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = str(exc) or "Interrupted"
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                result["error"] = result["error"] or f"Node cleanup: {exc}"
        if ros is not None and ros.ok():
            try:
                ros.shutdown()
            except Exception as exc:
                result["error"] = result["error"] or f"ROS cleanup: {exc}"
        if result["error"] is not None:
            result["success"] = False
        result["elapsed_seconds"] = time.monotonic() - started
        result["frames_saved"] = len(result["frames"])
        (out / "dataset.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in (
        "host", "scene", "frames_requested", "frames_saved", "images_received", "success", "error")},
        ensure_ascii=False), flush=True)
    print("DATASET_DIR=" + str(out), flush=True)
    return 0 if result["success"] and result["error"] is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
