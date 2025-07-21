import open3d as o3d
import struct
import numpy as np
import os

def visualize_with_open3d(points, camera_positions, f35_positions):
    if not points:
        print("no points to visualize")
        return

    pcd = o3d.geometry.PointCloud()
    xyz = np.array([(x, y, z) for x, y, z, _ in points], dtype=np.float64)
    pcd.points = o3d.utility.Vector3dVector(xyz)
    
    vals = np.array([val for _, _, _, val in points])
    norm_vals = vals / np.max(vals)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = norm_vals
    pcd.colors = o3d.utility.Vector3dVector(colors)

    cam_pcd = o3d.geometry.PointCloud()
    cam_xyz = np.array(camera_positions, dtype=np.float64)
    cam_pcd.points = o3d.utility.Vector3dVector(cam_xyz)
    cam_colors = np.zeros((len(camera_positions), 3))
    cam_colors[:, 1] = 1.0
    cam_pcd.colors = o3d.utility.Vector3dVector(cam_colors)

    f35_pcd = o3d.geometry.PointCloud()
    f35_xyz = np.array(f35_positions, dtype=np.float64)
    f35_pcd.points = o3d.utility.Vector3dVector(f35_xyz)
    f35_colors = np.zeros((len(f35_positions), 3))
    f35_colors[:, 0] = 1.0
    f35_pcd.colors = o3d.utility.Vector3dVector(f35_colors)

    output_dir = os.path.join(os.path.dirname(__file__), "visualize_output")
    os.makedirs(output_dir, exist_ok=True)
    o3d.io.write_point_cloud(os.path.join(output_dir, "voxel_grid.ply"), pcd)
    o3d.io.write_point_cloud(os.path.join(output_dir, "camera_positions.ply"), cam_pcd)
    o3d.io.write_point_cloud(os.path.join(output_dir, "f35_positions.ply"), f35_pcd)
    print(f"Saved visualization to {output_dir}/voxel_grid.ply, {output_dir}/camera_positions.ply, and {output_dir}/f35_positions.ply")

filename = 'sparse_voxel.bin'
threshold = 0
def read_sparse_voxel_bin(filename, value_threshold=0.0):
    points = []
    with open(filename, 'rb') as f:
        num_voxels_data = f.read(8)
        if len(num_voxels_data) != 8:
            raise ValueError("File too small or corrupted")
        num_voxels = struct.unpack('Q', num_voxels_data)[0]

        min_data = f.read(16)
        if len(min_data) != 16:
            raise ValueError("Incomplete extent data")
        min_x, min_y, min_z, voxel_size = struct.unpack('ffff', min_data)

        for _ in range(num_voxels):
            voxel_data = f.read(16)
            if len(voxel_data) != 16:
                raise ValueError("Incomplete voxel data")
            voxel_x, voxel_y, voxel_z, val = struct.unpack('iiif', voxel_data)
            if val > value_threshold:
                world_x = min_x + (voxel_x + 0.5) * voxel_size
                world_y = min_y + (voxel_y + 0.5) * voxel_size
                world_z = min_z + (voxel_z + 0.5) * voxel_size
                points.append((world_x, world_y, world_z, val))
    
    return points

points = read_sparse_voxel_bin(filename, threshold)
print(f"read {len(points)} voxels")

camera_positions = [
    (75.0, 0.0, 2.0),
    (23.1762752532959, 71.32923889160156, 2.0),
    (-60.676273345947266, 44.083892822265625, 2.0)
]

f35_positions = [
    (-625.0, 0.0, 1000.0),
    (-519.9580078125, 0.0, 1000.0),
    (-414.916015625, 0.0, 1000.0),
    (-309.87396240234375, 0.0, 1000.0)
]

visualize_with_open3d(points, camera_positions, f35_positions)