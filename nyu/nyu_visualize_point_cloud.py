import cv2
import open3d as o3d
import os
import numpy as np
import imageio

def parse_intrinsics(filepath, prefix):
    params = {}
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith(f'fx_{prefix}'):
                params['fx'] = float(line.split('=')[1].strip(' ;'))
            elif line.startswith(f'fy_{prefix}'):
                params['fy'] = float(line.split('=')[1].strip(' ;'))
            elif line.startswith(f'cx_{prefix}'):
                params['cx'] = float(line.split('=')[1].strip(' ;'))
            elif line.startswith(f'cy_{prefix}'):
                params['cy'] = float(line.split('=')[1].strip(' ;'))
    return params

def parse_extrinsics(filepath):
    R_vals = []
    t_x = t_y = t_z = None
    with open(filepath, 'r') as f:
        lines = f.readlines()
        in_R = False
        for line in lines:
            line = line.strip()
            if line.startswith('R = -['):
                in_R = True
                # Remove 'R = -[' and split the rest
                line = line.replace('R = -[', '')
            if in_R:
                # Remove trailing '];' or '...' and split
                line = line.replace('];', '').replace('...', '')
                nums = [x for x in line.split(',') if x.strip()]
                for num in nums:
                    try:
                        R_vals.append(float(num))
                    except ValueError:
                        pass
                # End of R block
                if '];' in line or len(R_vals) >= 9:
                    in_R = False
            if line.startswith('t_x'):
                t_x = float(line.split('=')[1].strip(' ;'))
            if line.startswith('t_y'):
                t_y = float(line.split('=')[1].strip(' ;'))
            if line.startswith('t_z'):
                t_z = float(line.split('=')[1].strip(' ;'))
    R = -np.array(R_vals).reshape(3, 3)
    t = np.array([t_x, t_y, t_z]).reshape(3, 1)
    return R, t

nyu_dir = 'nyu_depth_v2_labeled'
output_rgb_dir = os.path.join(nyu_dir, 'color')
output_depth_dir = os.path.join(nyu_dir, 'depth')
camera_params_file = os.path.join(nyu_dir, 'camera_params.m')

# Parse intrinsics and extrinsics
rgb_intr = parse_intrinsics(camera_params_file, 'rgb')
depth_intr = parse_intrinsics(camera_params_file, 'd')
R, t = parse_extrinsics(camera_params_file)

# Load images
depth = imageio.imread(os.path.join(output_depth_dir, '00000.png')).astype(np.float32) / 1000.0  # meters
rgb = imageio.imread(os.path.join(output_rgb_dir, '00000.png'))

h, w = depth.shape
fx_d, fy_d, cx_d, cy_d = depth_intr['fx'], depth_intr['fy'], depth_intr['cx'], depth_intr['cy']
fx_rgb, fy_rgb, cx_rgb, cy_rgb = rgb_intr['fx'], rgb_intr['fy'], rgb_intr['cx'], rgb_intr['cy']

points = []
colors = []

for v in range(h):
    for u in range(w):
        z = depth[v, u]
        if z == 0:
            continue
        # Backproject to depth camera space
        x = (u - cx_d) * z / fx_d
        y = (v - cy_d) * z / fy_d
        pt_d = np.array([[x], [y], [z]])
        # Transform to RGB camera space
        pt_rgb = R @ pt_d + t
        x_rgb, y_rgb, z_rgb = pt_rgb.flatten()
        # Project to RGB image
        u_rgb = int(round((x_rgb * fx_rgb) / z_rgb + cx_rgb))
        v_rgb = int(round((y_rgb * fy_rgb) / z_rgb + cy_rgb))
        if 0 <= u_rgb < rgb.shape[1] and 0 <= v_rgb < rgb.shape[0]:
            color = rgb[v_rgb, u_rgb, :] / 255.0
            points.append([x, y, z])
            colors.append(color)

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))
pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
o3d.visualization.draw_geometries([pcd])
