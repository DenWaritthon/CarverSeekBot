import matplotlib.pyplot as plt
import numpy as np
import cv2
import os

scripts_dir = os.path.dirname(os.path.abspath(__file__))

face_id = '000'
face_original_dir = os.path.abspath(os.path.join(scripts_dir, "..", "database", f'face_{face_id}_original.jpg'))
face_dir = os.path.abspath(os.path.join(scripts_dir, "..", "database", f'face_{face_id}.npy'))

# Load image (and convert BGR → RGB for plotting)
image = cv2.imread(face_original_dir)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Your 1D array
gradient_1d = np.load(face_dir, allow_pickle=True)

# Normalize if needed (to [0, 1])
gradient_1d = (gradient_1d - np.min(gradient_1d)) / (np.max(gradient_1d) - np.min(gradient_1d))

# Repeat vertically to make it visible (e.g., height 20)
gradient_img = np.tile(gradient_1d, (20, 1))  # shape: (20, 512)

# Match image width by resizing if necessary
if image.shape[1] != 512:
    gradient_img = cv2.resize(gradient_img, (image.shape[1], 20), interpolation=cv2.INTER_LINEAR)

# Plot both
fig, ax = plt.subplots(2, 1, figsize=(8, 6), gridspec_kw={'height_ratios': [9, 1]})

ax[0].imshow(image)
ax[0].axis("off")

ax[1].imshow(gradient_img, cmap='viridis', aspect='auto')  # You can use 'gray' or others
ax[1].axis("off")

plt.tight_layout()
plt.show()
