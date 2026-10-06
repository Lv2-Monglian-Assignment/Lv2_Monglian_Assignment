"""실행 중인 ROS 컬러 토픽에서 사진 한 장을 저장한다. 카메라는 실행하지 않는다."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True, help="새 촬영 폴더; 기존 폴더는 덮어쓰지 않음")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    if not args.topic or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("topic은 비어 있을 수 없으며 timeout은 양수인 유한값이어야 합니다.")

    out = Path(args.output).expanduser().resolve()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"기존 촬영을 보존합니다. 새 --output 폴더를 지정하세요: {out}", file=sys.stderr)
        return 2
    source = Path(__file__).read_bytes()
    (out / "capture_source.py").write_bytes(source)
    result = {
        "purpose": "Real ROS color snapshot for detection diagnosis; not accuracy evaluation",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "topic": args.topic,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "camera_launched_by_this_tool": False,
        "subscription_qos": "best_effort / keep_last depth 1 / volatile",
        "timeout_seconds": args.timeout,
        "source_sha256": hashlib.sha256(source).hexdigest(),
        "frames_received": 0,
        "success": False,
        "error": None,
    }
    ros = node = None
    try:
        import cv2
        from cv_bridge import CvBridge
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        ros = rclpy
        ros.init()
        node = ros.create_node("vision_color_snapshot_" + str(os.getpid()))
        received = []

        def receive(msg):
            if not received:
                received.append(msg)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        node.create_subscription(Image, args.topic, receive, qos)
        print(f"컬러 영상 한 장 수신 대기: {args.topic}", flush=True)
        deadline = time.monotonic() + args.timeout
        while not received and time.monotonic() < deadline:
            ros.spin_once(node, timeout_sec=min(.1, max(0.0, deadline - time.monotonic())))
        if not received:
            raise TimeoutError("제한시간 내 Image를 수신하지 못했습니다.")
        msg = received[0]
        result.update({
            "frames_received": 1,
            "header": {"stamp": {"sec": msg.header.stamp.sec,
                                    "nanosec": msg.header.stamp.nanosec},
                       "frame_id": msg.header.frame_id},
            "encoding": msg.encoding, "width": msg.width, "height": msg.height,
        })
        if msg.encoding not in ("rgb8", "bgr8") or msg.width <= 0 or msg.height <= 0:
            raise ValueError("rgb8/bgr8 컬러 Image와 양수 크기가 필요합니다.")
        bgr = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if bgr.shape != (msg.height, msg.width, 3) or bgr.dtype.name != "uint8":
            raise ValueError("변환된 영상 크기 또는 자료형이 올바르지 않습니다.")
        image_path = out / "color.png"
        if not cv2.imwrite(str(image_path), bgr):
            raise RuntimeError("PNG 저장 실패")
        result.update({
            "file_encoding": "BGR passed to OpenCV imwrite",
            "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
            "success": True,
        })
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = str(exc) or "Interrupted"
    finally:
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        (out / "capture.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    print("저장 폴더:", out, flush=True)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
