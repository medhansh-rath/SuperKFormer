import open3d as o3d
import numpy as np
import imageio
import sys
import os
from glob import glob

def write_organized_pcd(filename, points, colors, width, height):
    """
    Write an organized point cloud to a PCD file.
    Args:
        filename (str): The name of the output PCD file.
        points (np.ndarray): An Nx3 array of point coordinates.
        colors (np.ndarray): An Nx3 array of RGB colors.
        width (int): The width of the organized point cloud.
        height (int): The height of the organized point cloud.
    """
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {width}\n")
        f.write(f"HEIGHT {height}\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for i in range(width * height):
            x, y, z = points[i]
            r, g, b = (colors[i] * 255).astype(int)
            hex_str = "#{:02X}{:02X}{:02X}".format(r, g, b)
            int_value = int(hex_str.lstrip('#'), 16)
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {float(int_value):.1f}\n")

def create_organized_pcd_from_depth_rgb(scene_path, base_name):
    """
    Create an organized point cloud from depth and RGB images.
    """
    # Load depth and RGB images
    depth = imageio.imread(os.path.join(scene_path, "depth", f"{base_name}.png"))  # [Hd, Wd]
    color = imageio.imread(os.path.join(scene_path, "color", f"{base_name}.jpg"))    # [Hc, Wc, 3]
    intrinsics_depth = np.loadtxt('scannet_frames_25k/scene0000_00/intrinsics_depth.txt')
    intrinsics_color = np.loadtxt('scannet_frames_25k/scene0000_00/intrinsics_color.txt')

    fx_d, fy_d, cx_d, cy_d = intrinsics_depth[0,0], intrinsics_depth[1,1], intrinsics_depth[0,2], intrinsics_depth[1,2]
    fx_c, fy_c, cx_c, cy_c = intrinsics_color[0,0], intrinsics_color[1,1], intrinsics_color[0,2], intrinsics_color[1,2]

    Hd, Wd = depth.shape
    Hc, Wc, _ = color.shape

    points = []
    colors = []

    window_size = 300  # Use an odd number (e.g., 3, 5, 7)
    half_w = window_size // 2

    for v in range(Hd):
        for u in range(Wd):
            d = depth[v, u] / 1000.0  # meters
            if d == 0:
                # Search in a window around (v, u)
                for dv in range(-half_w, half_w + 1):
                    for du in range(-half_w, half_w + 1):
                        nv = v + dv
                        nu = u + du
                        if 0 <= nv < Hd and 0 <= nu < Wd:
                            candidate = depth[nv, nu]
                            if candidate > 0:
                                d = candidate / 1000.0
                                break
            # Backproject to 3D (depth camera)
            z = d
            x = (u - cx_d) * z / fx_d
            y = (v - cy_d) * z / fy_d

            # Project to color image
            u_c = int((fx_c * x / z) + cx_c)
            v_c = int((fy_c * y / z) + cy_c)
            if 0 <= u_c < Wc and 0 <= v_c < Hc:
                color_pixel = color[v_c, u_c]/255.0
                points.append([x, y, z])
                colors.append(color_pixel)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    write_organized_pcd(os.path.join(scene_path, "pcd", f"{base_name}.pcd"), 
                        np.array(points), 
                        np.array(colors), 
                        Wd, Hd)

    print(f"Created organized PCD file: {scene_path}/pcd/{base_name}.pcd")

if __name__ == "__main__":
    
    data_directory = "scannet_frames_25k"
    if not os.path.isdir(data_directory):
        print(f"Directory {data_directory} does not exist.")
        sys.exit(1)

    for scene in sorted(os.listdir(data_directory)):
        scene_path = os.path.join(data_directory, scene)
        if os.path.isdir(scene_path):
            print(f"Processing scene: {scene}")
            # Find all jpg and png files in the scene directory
            image_files = glob(os.path.join(scene_path, "color", "*.jpg"))
            if not image_files:
                print(f"No images found in {scene_path}. Skipping.")
                continue
            
            # Extract base names without extension and remove text after the last underscore
            base_names = [
                os.path.splitext(os.path.basename(f))[0].rsplit('_', 1)[0]
                for f in image_files
            ]
            base_names = list(set(base_names))

            if not os.path.exists(os.path.join(scene_path, "pcd")):
                os.makedirs(os.path.join(scene_path, "pcd"))

            for base_name in sorted(base_names):
                create_organized_pcd_from_depth_rgb(scene_path, base_name)
