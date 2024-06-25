import open3d as o3d

def load_and_visualize_mesh(file_path):
    # Load the triangle mesh
    mesh = o3d.io.read_triangle_mesh(file_path)
    
    # Check if the mesh is empty
    if mesh.is_empty():
        print("Failed to load mesh or the mesh is empty.")
        return
    
    # Compute the normal vectors of the mesh if not present
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    
    # Set up the visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Triangle Mesh Visualization", width=800, height=600)

    # Add the mesh to the visualizer
    vis.add_geometry(mesh)

    # Get the rendering options and disable back-face culling
    render_option = vis.get_render_option()
    render_option.mesh_show_back_face = True

    # Run the visualizer
    vis.run()
    vis.destroy_window()# Example usage

file_path = "mesh.ply"  # Replace this with the path to your mesh file
load_and_visualize_mesh(file_path)
