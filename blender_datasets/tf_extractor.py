import bpy
import mathutils
import json

# Specify the output file
output_file = "/home/harsha/Documents/MR/Project/camera_transforms.json"

# Get the active camera
camera = bpy.context.scene.camera

# Initialize the data dictionary
camera_transforms = []

# Iterate through each frame
for frame in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end + 1):
    bpy.context.scene.frame_set(frame)  # Set the current frame
    
    # Get camera position (x, y, z)
    position = camera.location
    
    # Get camera rotation in quaternions (w, x, y, z)
    # rotation = camera.rotation_quaternion
    rotation = camera.rotation_euler.to_quaternion()
    
    # Append the data with a timestamp (frame as time unit)
    camera_transforms.append({
        "frame": frame,
        "timestamp": frame / bpy.context.scene.render.fps,  # Time in seconds
        "position": [position.x, position.y, position.z],
        "rotation": [rotation.w, rotation.x, rotation.y, rotation.z]
    })
    
    print(position)
    print(rotation)

# Save data to JSON file
with open(output_file, "w") as f:
    json.dump(camera_transforms, f, indent=4)

print(f"Camera transforms exported to {output_file}")
