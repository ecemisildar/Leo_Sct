import cv2
import cv2.aruco as aruco
import os

# Choose ArUco dictionary: for example, DICT_4X4_50 allows 50 unique markers
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
marker_size = 200  # pixels, adjust as needed
output_folder = "aruco_markers"

os.makedirs(output_folder, exist_ok=True)

for marker_id in range(7):
    marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)
    filename = os.path.join(output_folder, f"aruco_marker_{marker_id}.png")
    cv2.imwrite(filename, marker_image)
    print(f"Saved marker ID {marker_id} to {filename}")
