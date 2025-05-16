import open3d as o3d
import numpy as np
import os

def create_rgb_and_depth_image_for_template_shapes(shape, output_directory='data', color=[0.2, 0.7, 0.3]):
    """
    Create RGB and depth images for a given shape and save them to the specified directory.
    Args:
        shape (str): The shape to create ('cylinder', 'cube', or 'icosahedron').
        output_directory (str): The directory to save the images.
        color (list): The RGB color for the shape.
    """
    if shape == 'cylinder':
        object = o3d.geometry.TriangleMesh.create_cylinder(radius=0.5, height=1.0, resolution=10000)
        object.rotate(object.get_rotation_matrix_from_xyz((3*np.pi / 4, 0, 0)), center=(0, 0, 0))
    elif shape == 'cube':
        object = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
        object.rotate(object.get_rotation_matrix_from_xyz((3*np.pi / 4, 0, np.pi / 4)), center=(0, 0, 0))
        object.translate((0, 1, 0))
    elif shape == 'icosahedron':
        object = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.5)
        object.rotate(object.get_rotation_matrix_from_xyz((0, np.pi/6, 0)), center=(0, 0, 0))

    # Set up visualizer
    object.compute_vertex_normals()
    object.paint_uniform_color(color)

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=1920, height=1080)
    vis.add_geometry(object)

    # Set camera position
    view_control = vis.get_view_control()
    view_control.set_front([0, 0, -1])
    view_control.set_lookat([0, 0, 0])
    view_control.set_up([0, -1, 0])
    view_control.set_zoom(1)
    if shape == 'cylinder':
        view_control.set_zoom(0.8)

    # Save intrinsic parameters to text file
    camera = view_control.convert_to_pinhole_camera_parameters()
    intrinsic = camera.intrinsic
    with open(os.path.join(output_directory, f"shape_intrinsics.txt"), 'w') as f:
        f.write(f"fx {intrinsic.intrinsic_matrix[0, 0]}\n")
        f.write(f"fy {intrinsic.intrinsic_matrix[1, 1]}\n")
        f.write(f"cx {intrinsic.intrinsic_matrix[0, 2]}\n")
        f.write(f"cy {intrinsic.intrinsic_matrix[1, 2]}\n")
        f.write(f"width {intrinsic.width}\n")
        f.write(f"height {intrinsic.height}")

    # Allow renderer to update
    vis.poll_events()
    vis.update_renderer()

    # Save RGB and depth images (depth in mm)
    os.makedirs(output_directory, exist_ok=True)
    vis.capture_screen_image(os.path.join(output_directory, f"{shape}_rgb.jpg"), do_render=True)
    vis.capture_depth_image(os.path.join(output_directory, f"{shape}_depth.png"), do_render=True, depth_scale=1000.0)
    vis.destroy_window()

def load_intrinsics(path):
    """
    Load camera intrinsics from a text file.
    Args:
        path (str): Path to the intrinsics text file.
    Returns:
        o3d.camera.PinholeCameraIntrinsic: Camera intrinsic parameters.
    """
    with open(path, 'r') as f:
        lines = f.readlines()
        fx = float(lines[0].split()[1])
        fy = float(lines[1].split()[1])
        cx = float(lines[2].split()[1])
        cy = float(lines[3].split()[1])
        width = int(lines[4].split()[1])
        height = int(lines[5].split()[1])
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return intrinsics
            
def save_point_cloud_with_attributes(pcd, intrinsic, image_name, directory='data'):
    """
    Save the point cloud with attributes (pixel coordinates, depth, RGB colors, normals) to a text file.
    Args:
        image_name (str): Name of the image for saving the file.
        directory (str): Directory to save the text file.
    """
    rgb_image = o3d.io.read_image(image_name + "_rgb.jpg")
    rgb_image = np.asarray(rgb_image)
    width = rgb_image.shape[1]
    height = rgb_image.shape[0]

    dtype = np.dtype([
    ('u', np.int32),
    ('v', np.int32),
    ('x', np.float64),
    ('y', np.float64),
    ('z', np.float64),
    ('r', np.float64),
    ('g', np.float64),
    ('b', np.float64),
    ('nx', np.float64),
    ('ny', np.float64),
    ('nz', np.float64)
    ])

    fixed_points = np.zeros((height, width), dtype=dtype)
    
    # Initialize the fixed points with default values
    fixed_points['u'] = np.arange(width)
    fixed_points['v'] = np.arange(height)[:, None]

    # Vectorized initialization of RGB values
    fixed_points['r'] = rgb_image[:, :, 0]/255.0
    fixed_points['g'] = rgb_image[:, :, 1]/255.0
    fixed_points['b'] = rgb_image[:, :, 2]/255.0

    # Project 3D points back to pixel coordinates (u, v) and compute depth (d)
    fx, fy = intrinsic.get_focal_length()
    cx, cy = intrinsic.get_principal_point()

    for i in range(np.asarray(pcd.points).shape[0]):
        x, y, z = pcd.points[i]
        nx, ny, nz = pcd.normals[i]
        if z > 0:  # Avoid division by zero
            u = int((x * fx) / z + cx)
            v = int((y * fy) / z + cy)
            fixed_points[int(v), int(u)]['x'] = x
            fixed_points[int(v), int(u)]['y'] = y
            fixed_points[int(v), int(u)]['z'] = z
            fixed_points[int(v), int(u)]['nx'] = nx
            fixed_points[int(v), int(u)]['ny'] = ny
            fixed_points[int(v), int(u)]['nz'] = nz

    # Flatten and write to file
    flat_points = fixed_points.reshape(-1)
    with open(os.path.join(directory, f"{image_name}_pcd.txt"), 'w') as f:
        f.write("u v d x y z r g b nx ny nz\n")  # Header
        for point in flat_points:
            f.write(
                f"{point['u']} {point['v']} {point['x']:.6f} {point['y']:.6f} {point['z']:.6f} "
                f"{point['r']:.6f} {point['g']:.6f} {point['b']:.6f} {point['nx']:.6f} {point['ny']:.6f} {point['nz']:.6f}\n"
            )

def create_custom_point_cloud(image_name, directory='data', visualize=False):
    """
    Create a point cloud from any RGB and depth images for the superpixel algorithm. Can also be used for template shapes.
    This function will create the RGB and depth images if they do not exist.
    Args:
        image_name (str): The name of the image (without extension).
        directory (str): The directory where the images are stored.
        visualize (bool): Whether to visualize the point cloud.
    """
    if not os.path.exists(os.path.join(directory, image_name + "_rgb.jpg")):
        if image_name not in ['cylinder', 'cube', 'icosahedron']:
            print(f"Image {image_name} does not exist in {directory}.")
            return
        else:
            create_rgb_and_depth_image_for_template_shapes(image_name, directory)
    
    # Load RGB and depth images
    rgb_image = o3d.io.read_image(os.path.join(directory, image_name + "_rgb.jpg"))
    depth_image = o3d.io.read_image(os.path.join(directory, image_name + "_depth.png"))
    if image_name in ['cylinder', 'cube', 'icosahedron']:
        intrinsic = load_intrinsics(os.path.join(directory, "shape_intrinsics.txt"))
    else:
        intrinsic = load_intrinsics(os.path.join(directory, "intrinsics.txt"))
    
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_image,
        depth_image,
        depth_scale=1000.0,
        convert_rgb_to_intensity=False
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd,
        intrinsic
    )
    if image_name in ['cylinder', 'cube', 'icosahedron']:
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
        pcd.orient_normals_towards_camera_location(camera_location=[0, 0, 0])  # or any point outside the cylinder
        normals = np.asarray(pcd.normals)
        pcd.normals = o3d.utility.Vector3dVector(-normals)
    else:
        pcd.estimate_normals()

    # Visualize the point cloud
    if(visualize):
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    save_point_cloud_with_attributes(pcd, intrinsic, image_name, directory)

# Example usage
if __name__ == "__main__":
    create_custom_point_cloud("cylinder")
    create_custom_point_cloud("cube")
    create_custom_point_cloud("icosahedron")
