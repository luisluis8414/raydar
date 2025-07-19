import bpy
import math
import json
import os

scene = bpy.context.scene

# Get all camera objects and sort
cameras = [obj for obj in bpy.data.objects if obj.type == 'CAMERA']
cameras.sort(key=lambda o: o.name)
cameras = cameras[:3]  # Use only first 3

# Get all moth armatures or objects (adjust filter if needed)
moths = [obj for obj in bpy.data.objects if obj.name.startswith("Moth")]

# Index cameras
cam_indices = {cam: idx for idx, cam in enumerate(cameras)}

# Prepare output directories
blend_dir = os.path.dirname(bpy.data.filepath)
output_dir = os.path.join(blend_dir, "..", "data", "moths", "frames")
os.makedirs(output_dir, exist_ok=True)

metadata = []

start_frame = scene.frame_start
end_frame = scene.frame_end
step = 10

current_step = 0

# Backup scene state
original_camera = scene.camera
original_filepath = scene.render.filepath
original_frame = scene.frame_current

# Backup render settings
original_color_mode = scene.render.image_settings.color_mode
original_color_depth = scene.render.image_settings.color_depth

# Set render settings to grayscale
scene.render.image_settings.color_mode = 'BW'  # Black and White
scene.render.image_settings.color_depth = '8'  # 8-bit depth for grayscale

for frame in range(start_frame, end_frame + 1, step):
    if current_step >= 2:
        break
    scene.frame_set(frame)
    
    for cam_obj in cameras:
        cam_index = cam_indices[cam_obj]
        frame_index = current_step
        
        scene.camera = cam_obj
        
        pos = cam_obj.location
        rot = cam_obj.rotation_euler
        cam_data = cam_obj.data
        fov_deg = math.degrees(cam_data.angle)
        
        image_file = f"camera{cam_index}_frame{frame_index:04d}.png"
        filepath = os.path.join(output_dir, image_file)
        
        scene.render.filepath = filepath
        bpy.ops.render.render(write_still=True)
        
        # Record positions of all moths
        moth_positions = {
            moth.name: {
                "X": float(moth.location.x),
                "Y": float(moth.location.y),
                "Z": float(moth.location.z)
            }
            for moth in moths
        }
        
        meta = {
            "camera_index": cam_index,
            "frame_index": frame_index,
            "camera_rotation": {
                "X": math.degrees(rot.x),
                "Y": math.degrees(rot.y),
                "Z": math.degrees(rot.z)
            },
            "fov_degrees": fov_deg,
            "image_file": f"data/moths/frames/{image_file}",
            "camera_position": {
                "X": float(pos.x),
                "Y": float(pos.y),
                "Z": float(pos.z)
            },
            "moth_positions": moth_positions
        }
        metadata.append(meta)
    
    current_step += 1

# Restore scene
scene.camera = original_camera
scene.render.filepath = original_filepath
scene.frame_set(original_frame)

# Restore render settings
scene.render.image_settings.color_mode = original_color_mode
scene.render.image_settings.color_depth = original_color_depth

# Write JSON
json_path = os.path.join(blend_dir, "..", "data", "moths", "metadata.json")
os.makedirs(os.path.dirname(json_path), exist_ok=True)
with open(json_path, 'w') as f:
    json.dump(metadata, f, indent=4)

print(f"Exported metadata to: {json_path}")
print(f"Exported images to: {output_dir}")