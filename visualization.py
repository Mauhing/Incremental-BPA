import open3d as o3d
import numpy as np

def load_and_visualize_mesh_and_points(mesh_file_path, points_file_path):
    # Load the triangle mesh
    mesh = o3d.io.read_triangle_mesh(mesh_file_path)
    
    # Check if the mesh is empty
    if mesh.is_empty():
        print("Failed to load mesh or the mesh is empty.")
        return
    
    # Compute the normal vectors of the mesh if not present
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    
    # Load ball centers from file, skipping the first line
    ball_centers = np.loadtxt(points_file_path, skiprows=1)
    
    # Create point cloud from ball centers
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(ball_centers)
    
    # Set up the visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Mesh and Ball Centers Visualization", width=800, height=600)

    # Add the mesh and point cloud to the visualizer
    vis.add_geometry(mesh)
    vis.add_geometry(pcd)

    # Get the rendering options and disable back-face culling
    render_option = vis.get_render_option()
    render_option.mesh_show_back_face = True
    
    # Set point size for better visibility
    render_option.point_size = 5.0

    # Run the visualizer
    vis.run()
    vis.destroy_window()

# Example usage
mesh_file_path = "mesh.ply"  # Replace this with the path to your mesh file
points_file_path = "ball_centers.txt"  # Path to the ball centers file
load_and_visualize_mesh_and_points(mesh_file_path, points_file_path)
