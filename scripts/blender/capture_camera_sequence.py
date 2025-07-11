import bpy
import os
import json
import math
import numpy as np

def setup_render_settings(fps=30):
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    bpy.context.scene.render.image_settings.color_mode = 'BW'
    bpy.context.scene.render.image_settings.color_depth = '8'
    bpy.context.scene.render.resolution_x = 1920
    bpy.context.scene.render.resolution_y = 1080
    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.render.fps = fps
    bpy.context.scene.render.engine = 'BLENDER_EEVEE_NEXT'

    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            space = area.spaces[0]
            space.shading.type = 'RENDERED'
            space.shading.use_scene_lights = True
            space.shading.use_scene_world = True
            space.shading.color_type = 'MATERIAL'
            space.overlay.show_overlays = False
            break

    bpy.context.scene.eevee.taa_render_samples = 64

    if bpy.context.scene.world:
        bpy.context.scene.world.use_nodes = True


def get_camera_rotation_euler(cam):
    rot = cam.rotation_euler
    return [math.degrees(angle) for angle in rot]

def get_camera_metadata(cam, frame_idx):
    cam_idx = int(cam.name.split('_')[1])
    fov_deg = math.degrees(cam.data.angle)
    rot_x, rot_y, rot_z = [math.degrees(a) for a in cam.rotation_euler]

    return {
        "camera_index":   cam_idx,
        "frame_index":    frame_idx,
        "yaw":   rot_z,   # Z  ➜ yaw
        "pitch": rot_x,   # X  ➜ pitch  (Rotation um X)
        "roll":  rot_y,   # Y  ➜ roll   (Rotation um Y)
        "fov_degrees":    fov_deg,
        "image_file":     f"camera{cam_idx}_frame{frame_idx:04d}.png",
        "camera_position":[cam.location.x,
                           cam.location.y,
                           cam.location.z]
}


def capture_frame_sequence(start_frame=1, end_frame=31):
    script_dir = bpy.path.abspath("//")
    base_dir = os.path.join(script_dir, "camera_sequence")
    frames_dir = os.path.join(base_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    cameras = [obj for obj in bpy.data.objects if obj.type == 'CAMERA']
    if not cameras:
        print("No cameras found in scene!")
        return

    cameras.sort(key=lambda x: int(x.name.split('_')[1]))
    original_camera = bpy.context.scene.camera
    all_metadata = []

    try:
        for cam in cameras:
            print(f"\nProcessing camera {cam.name}")
            bpy.context.scene.camera = cam
            cam_idx = int(cam.name.split('_')[1])

            for frame in range(start_frame, end_frame + 1, 30):
                bpy.context.scene.frame_set(frame)
                output_path = os.path.join(frames_dir, f"camera{cam_idx}_frame{frame:04d}.png")
                bpy.context.scene.render.filepath = output_path
                print(f"Rendering frame {frame}/{end_frame} for camera {cam_idx}")
                bpy.ops.render.render(write_still=True)

                metadata = get_camera_metadata(cam, frame)
                all_metadata.append(metadata)

    finally:
        bpy.context.scene.camera = original_camera
        metadata_path = os.path.join(base_dir, "metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(all_metadata, f, indent=4)
        print(f"\nMetadata saved to: {metadata_path}")
        print("Sequence capture complete!")

def main():
    setup_render_settings(fps=30)
    capture_frame_sequence(start_frame=1, end_frame=300)

if __name__ == "__main__":
    main()
