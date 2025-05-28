from ultralytics import YOLO
from insightface.app import FaceAnalysis
import cv2
import os
import numpy as np

# ---------- CONFIG ----------
SAVE_DIR = 'src/face_recognition/database'
os.makedirs(SAVE_DIR, exist_ok=True)

# Load YOLOv8-face and InsightFace
face_app = FaceAnalysis(name='buffalo_l', providers=['CUDAExecutionProvider'])
face_app.prepare(ctx_id=0)

# Start webcam
cap = cv2.VideoCapture(0)
print("Press 'c' to capture and register. Press 'q' to quit.")

face_id = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect faces using InsightFace
    faces = face_app.get(frame)

    # Draw face boxes
    for face in faces:
        x1, y1, x2, y2 = face.bbox.astype(int)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imshow("Register Face", frame)
    key = cv2.waitKey(1)

    if key == ord('c') and faces:
        emb = faces[0].embedding

        x1, y1, x2, y2 = [int(v) for v in faces[0].bbox]
        cropped_face = frame[y1:y2, x1:x2]

        # Optionally resize (e.g., to 112x112 like aligned faces)
        resized_face = cv2.resize(cropped_face, (112, 112))

        filename = f"face_{face_id:03d}.npy"
        filename_original = f"face_{face_id:03d}_original.jpg"
        cv2.imwrite(os.path.join(SAVE_DIR, filename_original), resized_face)
        np.save(os.path.join(SAVE_DIR, filename), emb)
        
        print(f"[INFO] Saved embedding as {filename}")
        face_id += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()