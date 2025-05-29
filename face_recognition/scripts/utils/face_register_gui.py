import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import cv2
import os
import numpy as np
from insightface.app import FaceAnalysis

# ---------- CONFIG ----------
SAVE_DIR = 'src/face_recognition/database'
os.makedirs(SAVE_DIR, exist_ok=True)

face_app = FaceAnalysis(name='buffalo_l', providers=['CUDAExecutionProvider'])
face_app.prepare(ctx_id=0)

class FaceRegisterApp:
    def __init__(self, window):
        self.window = window
        self.window.title("Face Registration")

        self.cap = None
        self.running = False
        self.frame = None
        self.faces = []

        self.video_label = tk.Label(self.window)
        self.video_label.pack()

        self.id_label = tk.Label(self.window, text="Enter Face ID:")
        self.id_label.pack()

        self.face_id_entry = tk.Entry(self.window)
        self.face_id_entry.pack()

        self.capture_button = tk.Button(self.window, text="Capture Face", command=self.capture_face)
        self.capture_button.pack()

        self.start_button = tk.Button(self.window, text="Start Camera", command=self.start_camera)
        self.start_button.pack()

        self.quit_button = tk.Button(self.window, text="Quit", command=self.quit_app)
        self.quit_button.pack()

    def start_camera(self):
        if not self.running:
            self.cap = cv2.VideoCapture(0)
            self.running = True
            self.update_frame()

    def update_frame(self):
        if self.running:
            ret, frame = self.cap.read()
            if not ret:
                return

            self.frame = frame.copy()
            self.faces = face_app.get(frame)

            for face in self.faces:
                x1, y1, x2, y2 = face.bbox.astype(int)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(img_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            self.video_label.imgtk = img_tk
            self.video_label.configure(image=img_tk)

            self.window.after(10, self.update_frame)

    def capture_face(self):
        face_id_text = self.face_id_entry.get().strip()
        if not face_id_text:
            messagebox.showwarning("Input Error", "Please enter a face ID before capturing.")
            return

        if self.frame is not None and self.faces:
            emb = self.faces[0].embedding
            x1, y1, x2, y2 = [int(v) for v in self.faces[0].bbox]
            cropped = self.frame[y1:y2, x1:x2]
            resized = cv2.resize(cropped, (112, 112))

            filename = f"face_{face_id_text}.npy"
            filename_img = f"face_{face_id_text}_original.jpg"

            np.save(os.path.join(SAVE_DIR, filename), emb)
            cv2.imwrite(os.path.join(SAVE_DIR, filename_img), resized)
            print(f"[INFO] Saved face with ID: {face_id_text}")
            messagebox.showinfo("Saved", f"Saved face with ID: {face_id_text}")
        else:
            messagebox.showwarning("No Face", "No face detected in the frame.")

    def quit_app(self):
        self.running = False
        if self.cap:
            self.cap.release()
        self.window.destroy()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    root = tk.Tk()
    app = FaceRegisterApp(root)
    root.mainloop()
