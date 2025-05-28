from ultralytics import YOLO
from insightface.app import FaceAnalysis
import cv2
import os
import numpy as np
from numpy.linalg import norm
from deep_sort_realtime.deepsort_tracker import DeepSort

def cosine_similarity(vec1, vec2):
    return np.dot(vec1, vec2) / (norm(vec1) * norm(vec2) + 1e-6)

def match_face(embedding, known_faces, threshold=0.5):
    similarity = cosine_similarity(embedding, known_faces)

    if similarity >= threshold:
        return "Matched Face", similarity
    else:
        return "Unknown", similarity

# ---------- CONFIG ----------
DATA_DIR = 'src/face_recognition/database'
face_id = '000'
focused_classes = 0 # person
confidence_threshold = 0.5

scripts_dir = os.path.dirname(os.path.abspath(__file__))
face_dir = os.path.abspath(os.path.join(scripts_dir, "..", "database", f'face_{face_id}.npy'))
emb_to_match = np.load(face_dir, allow_pickle=True)

# Load YOLOv8-face and InsightFace
yolo_model = YOLO('src/face_recognition/model/yolo11n.pt')
tracker = DeepSort(max_age=30)
face_app = FaceAnalysis(name='buffalo_l', providers=['CUDAExecutionProvider'])
face_app.prepare(ctx_id=0)

# Start webcam
cap = cv2.VideoCapture(0)
print("Press 'c' to capture and register. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection
    results = yolo_model(frame)[0]

    detections = []

    # Collect only person class detections
    for box in results.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        # Filter the detections based on class ID and confidence
        if cls_id == focused_classes and conf > confidence_threshold: 

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append(([x1, y1, x2 - x1, y2 - y1], conf, 'person'))

    # Track using Deep SORT
    tracks = tracker.update_tracks(detections, frame=frame)

    # Draw results on frame
    for track in tracks:
        if not track.is_confirmed():
            continue
        track_id = track.track_id

        x1, y1, x2, y2 = map(int, track.to_ltrb())
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'ID {track_id}', (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
    
    cv2.imshow("Face Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()