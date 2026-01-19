import open3d as o3d
import copy
import numpy as np
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Apply sequential rotations to a PLY file")
    parser.add_argument("file", help="Input PLY file")
    parser.add_argument("-o", "--output", default="final_rotated_tank.ply", help="Output filename")
    parser.add_argument("--merge", action="store_true", help="Merge the rotated copies with the original (Generate geometry)")
    
    args = parser.parse_args()
    
    # 1. Load
    print(f"Loading {args.file}...")
    pcd = o3d.io.read_point_cloud(args.file)
    center = pcd.get_center()
    pcd.translate(-center) # Center for rotation
    print("Centered object at (0,0,0)")
    
    # 2. Apply Rotations
    # User Request: "180 degree horizontally (Y) and then 180 vertically (X?)"
    
    # Rotation 1: Horizontal (Y-axis)
    R_hor = pcd.get_rotation_matrix_from_xyz((0, np.pi, 0)) # 180 deg Y
    
    # Rotation 2: Vertical (X-axis) - Interpreting vertical rotation as flip relative to ground
    R_ver = pcd.get_rotation_matrix_from_xyz((np.pi, 0, 0)) # 180 deg X
    
    if args.merge:
        # Option A: Merge logic (Symmetry expansion)
        # 1. Original
        # 2. Rotated Horizontal
        # 3. Rotated Vertical (from Original? or from Horizontal?)
        # Let's assume full symmetry expansion: Org + RotH + RotV + RotH*RotV
        print("Mode: MERGE (Generating Symmetry)")
        
        pcd_h = copy.deepcopy(pcd)
        pcd_h.rotate(R_hor, center=(0,0,0))
        
        pcd_v = copy.deepcopy(pcd)
        pcd_v.rotate(R_ver, center=(0,0,0))
        
        pcd_hv = copy.deepcopy(pcd_h) # Horizontally rotated, then Vertically
        pcd_hv.rotate(R_ver, center=(0,0,0))
        
        # Colors
        pcd.paint_uniform_color([1, 0.7, 0])      # Gold
        pcd_h.paint_uniform_color([0, 0.6, 0.8])  # Blue
        pcd_v.paint_uniform_color([0.8, 0, 0.2])  # Red
        pcd_hv.paint_uniform_color([0, 0.8, 0.2]) # Green
        
        final_pcd = pcd + pcd_h + pcd_v + pcd_hv
        
    else:
        # Option B: Transform logic (Re-orientation)
        print("Mode: TRANSFORM (Re-orienting)")
        final_pcd = copy.deepcopy(pcd)
        
        # Apply 180 Horizontal
        final_pcd.rotate(R_hor, center=(0,0,0))
        print("Applied 180 degree Horizontal rotation (Y-axis)")
        
        # Apply 180 Vertical
        final_pcd.rotate(R_ver, center=(0,0,0))
        print("Applied 180 degree Vertical rotation (X-axis)")
        
    # 3. Save
    print(f"Saving to {args.output}...")
    o3d.io.write_point_cloud(args.output, final_pcd)
    
    # 4. Visualize
    print("Visualizing...")
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    o3d.visualization.draw_geometries([final_pcd, axes], window_name="Rotated Tank")

if __name__ == "__main__":
    main()
