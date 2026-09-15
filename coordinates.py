import cv2
import numpy as np
from lidar import get_x_axis

# === Define keypoints ===
KPT_NAMES = ['stem_tip', 'stem_tip_2', 'stem_base_2', 'stem_base']
STEM_TIP_2_IDX = KPT_NAMES.index('stem_tip_2')

# === Load camera calibration data (intrinsics) ===
try:
    calib_data = np.load("calibration/camera_calibration.npz")
    mtx = calib_data["mtx"]          # Camera matrix
    dist = calib_data["dist"]        # Distortion coefficients
    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]
    print("Camera calibration loaded successfully.")
except Exception as e:
    print(f"Warning: Could not load calibration file: {e}")
    fx = fy = 1
    cx = cy = 0


def get_coordinates(stem_tip_2_coords, frame):
    """
    Extract (X, Y) in millimeters for 'stem_tip_2' from the given pixel coordinates.

    Parameters:
        stem_tip_2_coords: (x_pix, y_pix) - Pixel coordinates of the stem_tip_2 keypoint
        frame: The frame captured by the camera (needed for calibration).

    Returns:
        coords_list: [{'stem_tip_2': (X_mm, Y_mm)}]
    """
    coords_list = []

    if stem_tip_2_coords is None:
        print("Error: No coordinates provided.")
        return coords_list  # Return an empty list

    x_pix, y_pix = stem_tip_2_coords

    # Convert to world coordinates (in mm)
    Z_mm = 125  # Example Z value (depth)
    X_mm = (x_pix - cx) * Z_mm / fx
    Y_mm = (y_pix - cy) * Z_mm / fy
    
    coords_list.append({'stem_tip_2': (float(X_mm), float(Y_mm), float(Z_mm))})

    return coords_list

# Example of how this function is used:
if __name__ == "__main__":
    # Assuming 'stem_tip_2_coords' is the output from detect_pepper()
    stem_tip_2_coords = (320, 240)  # Example pixel coordinates
    frame = cv2.imread("image.jpg")  # Replace with the actual frame or image

    # Now call the get_coordinates function with the stem_tip_2 coordinates
    coords = get_coordinates(stem_tip_2_coords, frame)

    print(f"Real-world coordinates of stem_tip_2: {coords}")
