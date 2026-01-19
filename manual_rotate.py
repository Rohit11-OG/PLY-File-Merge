import open3d as o3d
import copy
import numpy as np
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Rotate a specific PLY file 180 degrees around an axis")
    parser.add_argument("file", help="Input PLY file to rotate")
    parser.add_argument("-o", "--output", default="rotated_file.ply", help="Output filename")
    parser.add_argument("--axis", required=True, choices=["x", "y", "z"], help="Axis to rotate 180 degrees around (x, y, or z)")
    parser.add_argument("--visualize", action="store_true", help="Visualize result")
    
    args = parser.parse_args()
    
    # 1. Load
    print(f"Loading {args.file}...")
    pcd = o3d.io.read_point_cloud(args.file)
    
    # Center? 
    # Usually manual rotation for alignment requires rotating around the center of the object.
    center = pcd.get_center()
    pcd.translate(-center)
    print("Centered object at (0,0,0) for stable rotation")
    
    # 2. Rotate
    print(f"Applying 180-degree rotation around {args.axis.upper()}-axis...")
    
    if args.axis == "x":
        R = pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    elif args.axis == "y":
        R = pcd.get_rotation_matrix_from_xyz((0, np.pi, 0))
    else: # z
        R = pcd.get_rotation_matrix_from_xyz((0, 0, np.pi))
        
    pcd.rotate(R, center=(0,0,0))
    
    # 3. Save
    print(f"Saving to {args.output}...")
    o3d.io.write_point_cloud(args.output, pcd)
    
    # 4. Visualize
    if args.visualize:
        print("Visualizing...")
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
        o3d.visualization.draw_geometries([pcd, axes], window_name=f"Rotated {args.axis.upper()} 180")

if __name__ == "__main__":
    main()
