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
        f.write("TYPE F F F U\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {width}\n")
        f.write(f"HEIGHT {height}\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {width * height}\n")
        f.write("DATA ascii\n")
        for i in range(width * height):
            x, y, z = points[i]
            r, g, b = (colors[i] * 255).astype(np.uint8)
            rgb = (r << 16) | (g << 8) | b
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {rgb}\n")

def create_organized_pcd_from_depth_rgb(base_name):
    """
    Create an organized point cloud from depth and RGB images.
    """
    # Load depth and RGB images
    depth = imageio.imread(f"{base_name}_depth.png")
    rgb = imageio.imread(f"{base_name}_rgb.jpg")

    # Camera intrinsics
    fx, fy = 935.31, 935.31
    cx, cy = 959.5, 539.5

    height, width = depth.shape
    points = []
    colors = []

    for v in range(height):
        for u in range(width):
            d = depth[v, u] / 1000.0  # convert to meters if needed
            if d == 0:  # skip invalid points
                points.append([0, 0, 0])
                colors.append(rgb[v, u] / 255.0)
                continue
            z = d
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points.append([x, y, z])
            colors.append(rgb[v, u] / 255.0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

    write_organized_pcd(f"{base_name}.pcd", np.array(points), np.array(colors), width, height)
    print(f"Created organized PCD file: {base_name}.pcd")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python create_organized_pcd.py <data_directory>")
        sys.exit(1)
    
    data_directory = sys.argv[1]
    if not os.path.isdir(data_directory):
        print(f"Directory {data_directory} does not exist.")
        sys.exit(1)

    # Find all jpg and png files
    image_files = glob(os.path.join(data_directory, "*.jpg"))

    # Extract base names without extension and remove text after the last underscore
    base_names = [
        os.path.splitext(os.path.basename(f))[0].rsplit('_', 1)[0]
        for f in image_files
    ]
    base_names = list(set(base_names))  # Remove duplicates

    for base_name in base_names:
        create_organized_pcd_from_depth_rgb(os.path.join(data_directory, base_name))
