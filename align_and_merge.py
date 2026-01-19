import open3d as o3d
import copy
import numpy as np
import argparse

def align_to_principal_axes(pcd):
    """
    Aligns the point cloud to the world axes using OBB (Oriented Bounding Box).
    This ensures the object is straight (not tilted) before merging.
    """
    # 1. Center
    center = pcd.get_center()
    pcd.translate(-center)
    
    # 2. Compute OBB
    obb = pcd.get_oriented_bounding_box()
    R = obb.R # Rotation matrix of the OBB
    
    # 3. Rotate object to align OBB with World
    # We invert the rotation to bring the object to axis-aligned state
    pcd.rotate(np.linalg.inv(R), center=(0,0,0))
    
    return pcd

def main():
    parser = argparse.ArgumentParser(description="Auto-align object to XYZ axes and merge symmetrically")
    parser.add_argument("file", help="Input PLY file")
    parser.add_argument("-o", "--output", default="aligned_symmetric_tank.ply", help="Output filename")
    parser.add_argument("--axis", default="y", choices=["x", "y", "z"], help="Symmetry axis for merging (default: y)")
    parser.add_argument("--visualize", action="store_true", help="Visualize result")
    
    args = parser.parse_args()
    
    # 1. Load
    print(f"Loading {args.file}...")
    pcd = o3d.io.read_point_cloud(args.file)
    original_count = len(pcd.points)
    
    # 2. Auto-Align
    print("Auto-aligning object to Principal Axes (fixing tilt)...")
    pcd_aligned = align_to_principal_axes(pcd)
    
    # 3. Symmetric Merge
    # Create copy, rotate 180 deg around selected axis
    print(f"Applying 180-degree symmetry merge around {args.axis}-axis...")
    pcd_mirrored = copy.deepcopy(pcd_aligned)
    
    if args.axis == "x":
        R_sym = pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    elif args.axis == "y":
        R_sym = pcd.get_rotation_matrix_from_xyz((0, np.pi, 0))
    else: # z
        R_sym = pcd.get_rotation_matrix_from_xyz((0, 0, np.pi))
        
    pcd_mirrored.rotate(R_sym, center=(0,0,0))
    
    # 4. Colorize
    pcd_aligned.paint_uniform_color([1, 0.7, 0])      # Gold (Original Aligned)
    pcd_mirrored.paint_uniform_color([0, 0.6, 0.8])   # Blue (Mirrored)
    
    merged = pcd_aligned + pcd_mirrored
    
    # 5. Save
    print(f"Saving to {args.output}...")
    o3d.io.write_point_cloud(args.output, merged)
    
    # 6. Visualize
    if args.visualize:
        print("Visualizing...")
        # Grid/Axes
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
        
        # Draw basic grid for "Plan" view
        # (This is just visual help)
        
        o3d.visualization.draw_geometries([merged, axes], 
                                          window_name="Aligned Symmetric Merge",
                                          width=1024, height=768)

if __name__ == "__main__":
    main()
