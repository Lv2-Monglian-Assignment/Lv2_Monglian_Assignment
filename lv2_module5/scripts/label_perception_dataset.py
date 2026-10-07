"""Annotate an unlabelled scene after independent target/scene confirmation; preserve raw metadata."""

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--scene", choices=("present", "absent"), required=True)
    ap.add_argument("--basis", required=True, help="Target identity and original-image review evidence")
    ap.add_argument("--output-name", default="dataset-labelled.json")
    args = ap.parse_args()
    if not args.basis.strip() or Path(args.output_name).name != args.output_name:
        ap.error("A physical/visual confirmation basis and plain output filename are required")
    source_path = Path(args.dataset).resolve()
    raw = source_path.read_bytes()
    original = json.loads(raw)
    if (original.get("scene") != "unlabelled" or not original.get("success") or original.get("error")
            or not original.get("frames") or any(f.get("target_present") is not None for f in original["frames"])):
        ap.error("Only a complete, unlabelled original dataset can be annotated")
    data = copy.deepcopy(original)
    data.update(scene=args.scene, label_basis=args.basis)
    for frame in data["frames"]:
        frame["target_present"] = args.scene == "present"
    data["annotation"] = {"source_dataset": source_path.name,
                          "source_dataset_sha256": hashlib.sha256(raw).hexdigest(),
                          "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
                          "label_basis": args.basis,
                          "labeling_tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    out = source_path.parent / args.output_name
    with out.open("x") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(f"LABELLED_MANIFEST={out}; raw dataset preserved")


if __name__ == "__main__":
    main()
