"""Run YOLOv8 inference on the ISS cubesat video."""

from __future__ import annotations

from pathlib import Path

import cv2
from tqdm import tqdm
from ultralytics import YOLO


def annotate_video() -> None:
    base_dir = Path(__file__).resolve().parent
    model_path = base_dir / "best.pt"
    video_path = base_dir / "iss_cubesat_orbit.mp4"
    output_path = base_dir / "output_detected.mp4"

    if not model_path.exists():
        raise FileNotFoundError(f"Model weights not found: {model_path}")
    if not video_path.exists():
        raise FileNotFoundError(f"Input video not found: {video_path}")

    model = YOLO(str(model_path))

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or None

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))

    try:
        progress = tqdm(total=total_frames, desc="Processing frames", unit="frame")
        names = model.names

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            results = model.predict(frame, imgsz=640, conf=0.25, verbose=False)
            annotated = frame

            if results:
                boxes = results[0].boxes
                if boxes is not None and boxes.xyxy is not None:
                    xyxy = boxes.xyxy.cpu().numpy()
                    classes = boxes.cls.cpu().numpy()
                    confidences = boxes.conf.cpu().numpy()

                    for (x1, y1, x2, y2), cls_id, conf in zip(xyxy, classes, confidences):
                        label = names.get(int(cls_id), str(int(cls_id))) if isinstance(names, dict) else str(int(cls_id))
                        color = (0, 255, 0) if label.lower() == "satellite" else (0, 128, 255)
                        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                        text = f"{label} {conf:.2f}"
                        cv2.putText(annotated, text, (int(x1), max(int(y1) - 10, 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

            writer.write(annotated)
            progress.update(1)
    finally:
        cap.release()
        writer.release()
        if 'progress' in locals():
            progress.close()

    print(f"Annotated video saved to: {output_path}")


if __name__ == "__main__":
    annotate_video()
