import xml.etree.ElementTree as ET
import json

# Path to your SDF file
sdf_file = "/home/ecem/ros2_ws/src/leo_simulator-ros2/leo_gz_worlds/worlds/tugbot_depot.sdf"

# Path to save the JSON
json_file = "cylinder_positions.json"

# Parse the SDF file
tree = ET.parse(sdf_file)
root = tree.getroot()

# Array to store (x, y) positions
cylinder_positions = []

# Iterate over all models
for model in root.findall('.//model'):
    name = model.get('name', '')
    if "cylinder" in name.lower():  # case-insensitive match
        pose = model.find('pose')
        if pose is not None:
            # Pose format: "x y z roll pitch yaw"
            pose_values = pose.text.strip().split()
            x = float(pose_values[0])
            y = float(pose_values[1])
            cylinder_positions.append([x, y])  # save as list [x, y]

# Save array to JSON
with open(json_file, "w") as f:
    json.dump(cylinder_positions, f)

print(f"Saved {len(cylinder_positions)} cylinder (x, y) positions to {json_file}")
