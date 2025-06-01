#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from insightface.app import FaceAnalysis
from numpy.linalg import norm
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from std_msgs.msg import String

def cosine_similarity(vec1, vec2):
    return np.dot(vec1, vec2) / (norm(vec1) * norm(vec2) + 1e-6)

class FaceMatcherNode(Node):
    def __init__(self):
        super().__init__('face_match_node')

        # --- Configuration ---
        self.face_id = '002'
        scripts_dir = os.path.dirname(os.path.abspath(__file__))
        self.emb_path = os.path.abspath(os.path.join(scripts_dir, "..", "..", "share", "face_recognition", "database", f"face_{self.face_id}.npy"))

        if not os.path.exists(self.emb_path):
            self.get_logger().error(f"Embedding file not found: {self.emb_path}")
            rclpy.shutdown()
            return

        self.emb_known = np.load(self.emb_path, allow_pickle=True)

        # Initialize face detector
        self.face_app = FaceAnalysis(name='buffalo_l', providers=['CUDAExecutionProvider'])
        self.face_app.prepare(ctx_id=0)

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Camera could not be opened.")
            rclpy.shutdown()
            return

        self.get_logger().info("Running Face Matcher. Press 'q' to quit.")
        self.timer = self.create_timer(0.1, self.process_frame)  # ~30 FPS
        self.matched_client = self.create_client(SetBool, f"/face_matched")
        # self.matched_pub = self.create_publisher(Bool, '/face_matched', 10)
        self.img_pub = self.create_publisher(Image, '/camera/image', 10)

        self.change_face_srv = self.create_service(
            String, 
            'change_face_id', 
            self.change_face_id_callback
        )

        self.bridge = CvBridge()
        self.state = False

    def set_matched_request(self, status: bool)->None:
        req = SetBool.Request()
        req.data = status
        self.matched_client.call_async(req) 

    def match_face(self, embedding, threshold=0.5):
        similarity = cosine_similarity(embedding, self.emb_known)
        if similarity >= threshold:
            return "Matched Face", similarity
        else:
            return "Unknown", similarity

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame.")
            return
        
        faces = self.face_app.get(frame)

        face = faces[np.random.randint(0, len(faces))] if faces else None

        if face is not None:
            x1, y1, x2, y2 = face.bbox.astype(int)
            embedding = face.embedding
            name, score = self.match_face(embedding)

            if name == "Matched Face":
                self.state = True
                self.set_matched_request(True)
            else:
                if self.state:
                    self.state = False
                    self.set_matched_request(False)

            # Draw and label
            label = f"{name} ({score:.2f})"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.img_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def change_face_id_callback(self, request, response):
        """Service callback to change face_id"""
        new_face_id = request.data.strip()
        
        if not new_face_id:
            response.data = f"Error: Empty face_id provided"
            return response
        
        old_face_id = self.face_id
        self.face_id = new_face_id
        
        # Try to load the new embedding
        if self.load_face_embedding():
            response.data = f"Successfully changed face_id from '{old_face_id}' to '{new_face_id}'"
            self.get_logger().info(f"Face ID changed to: {new_face_id}")
        else:
            # Revert to old face_id if loading fails
            self.face_id = old_face_id
            self.load_face_embedding()
            response.data = f"Error: Could not load embedding for face_id '{new_face_id}'. Reverted to '{old_face_id}'"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FaceMatcherNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
