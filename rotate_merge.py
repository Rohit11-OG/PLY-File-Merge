import open3d as o3d
import copy
import numpy as np
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Rotate a PLY file 180 degrees and merge with original")
    parser.add_argument("file", help="Input PLY file")
    parser.add_argument("-o", "--output", default="rotated_merge.ply", help="Output filename")
    parser.add_argument("--axis", default="y", choices=["x", "y", "z"], help="Rotation axis (default: y)")
    parser.add_argument("--visualize", action="store_true", help="Visualize result")
    
    args = parser.parse_args()
    
    # 1. Load Original
    print(f"Loading {args.file}...")
    pcd = o3d.io.read_point_cloud(args.file)
    
    # Center the original first (crucial for rotation)
    center = pcd.get_center()
    pcd.translate(-center)
    print(f"Centered point cloud at (0,0,0)")

    # 2. Create Rotated Copy
    pcd_rotated = copy.deepcopy(pcd)
    
    # Define rotation matrix (180 degrees = pi radians)
    # Using Euler angles for simplicity
    if args.axis == "x":
        R = pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    elif args.axis == "y":
        R = pcd.get_rotation_matrix_from_xyz((0, np.pi, 0))
    else: # z
        R = pcd.get_rotation_matrix_from_xyz((0, 0, np.pi))
        
    pcd_rotated.rotate(R, center=(0,0,0))
    print(f"Created copy rotated 180 degrees around {args.axis}-axis")
    
    # 3. Merge
    pcd.paint_uniform_color([1, 0.7, 0]) # Original = Gold
    pcd_rotated.paint_uniform_color([0, 0.6, 0.8]) # Rotated = Blue
    
    merged = pcd + pcd_rotated
    
    # 4. Save
    print(f"Saving to {args.output}...")
    o3d.io.write_point_cloud(args.output, merged)
    
    # 5. Visualize
    if args.visualize:
        print("Visualizing... (Gold=Original, Blue=Rotated)")
        # Add coordinate frame
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
        o3d.visualization.draw_geometries([merged, axes], window_name="Symmetric Merge")

if __name__ == "__main__":
    main()
