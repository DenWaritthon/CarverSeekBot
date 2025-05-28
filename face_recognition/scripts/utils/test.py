from insightface.app import FaceAnalysis
import cv2
import os
import numpy as np
from numpy.linalg import norm

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

scripts_dir = os.path.dirname(os.path.abspath(__file__))
face_dir = os.path.abspath(os.path.join(scripts_dir, "..", "database", f'face_{face_id}.npy'))
emb_to_match = np.load(face_dir, allow_pickle=True)

# Load YOLOv8-face and InsightFace
face_app = FaceAnalysis(name='buffalo_l', providers=['CUDAExecutionProvider'])
face_app.prepare(ctx_id=0)

# Start webcam
cap = cv2.VideoCapture(0)
print("Press 'c' to capture and register. Press 'q' to quit.")

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

        embedding = face.embedding

        # Match to known faces
        name, score = match_face(embedding, emb_to_match)

        # Display result
        label = f"{name} ({score:.2f})"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Face Matching", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()