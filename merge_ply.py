#!/usr/bin/env python3
"""
PLY Point Cloud Merger
----------------------
Merge multiple PLY files into a single point cloud.

Usage:
    python merge_ply.py file1.ply file2.ply file3.ply ... -o output.ply
    python merge_ply.py *.ply -o merged.ply
    python merge_ply.py file1.ply file2.ply -o output.ply --voxel 0.01
"""

import open3d as o3d
import argparse
import sys
import os


def merge_point_clouds(ply_files, output_file, voxel_size=None, visualize=False):
    """
    Merge multiple PLY files into one point cloud.
    
    Args:
        ply_files: List of paths to PLY files
        output_file: Path for the merged output PLY file
        voxel_size: Optional voxel size for downsampling (None = no downsampling)
        visualize: Whether to show the merged point cloud
    """
    if len(ply_files) < 2:
        print("Error: Please provide at least 2 PLY files to merge.")
        sys.exit(1)
    
    print(f"Merging {len(ply_files)} PLY files...")
    print("-" * 50)
    
    # Load and merge all point clouds
    merged_pcd = o3d.geometry.PointCloud()
    total_original_points = 0
    
    for i, ply_file in enumerate(ply_files, 1):
        if not os.path.exists(ply_file):
            print(f"Error: File not found - {ply_file}")
            sys.exit(1)
        
        pcd = o3d.io.read_point_cloud(ply_file)
        point_count = len(pcd.points)
        total_original_points += point_count
        
        print(f"  [{i}] {os.path.basename(ply_file)}: {point_count:,} points")
        merged_pcd += pcd
    
    print("-" * 50)
    print(f"Total points before merge: {total_original_points:,}")
    
    # Optional: Voxel downsampling to remove duplicates and reduce density
    if voxel_size is not None and voxel_size > 0:
        print(f"Applying voxel downsampling (voxel size: {voxel_size})...")
        merged_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"Points after downsampling: {len(merged_pcd.points):,}")
    else:
        print(f"Total points in merged cloud: {len(merged_pcd.points):,}")
    
    # Save the result
    o3d.io.write_point_cloud(output_file, merged_pcd)
    print(f"\n✅ Saved merged point cloud to: {output_file}")
    
    # Optional visualization
    if visualize:
        print("\nOpening visualization... (close window to exit)")
        o3d.visualization.draw_geometries([merged_pcd], 
                                          window_name="Merged Point Cloud",
                                          width=1280, height=720)
    
    return merged_pcd


def main():
    parser = argparse.ArgumentParser(
        description="Merge multiple PLY point cloud files into one.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python merge_ply.py part1.ply part2.ply part3.ply -o merged.ply
  python merge_ply.py *.ply -o output.ply --voxel 0.005
  python merge_ply.py scan1.ply scan2.ply -o result.ply --visualize
        """
    )
    
    parser.add_argument("files", nargs="+", help="PLY files to merge")
    parser.add_argument("-o", "--output", default="merged_output.ply",
                        help="Output filename (default: merged_output.ply)")
    parser.add_argument("-v", "--voxel", type=float, default=None,
                        help="Voxel size for downsampling (e.g., 0.005 for 5mm)")
    parser.add_argument("--visualize", action="store_true",
                        help="Show the merged point cloud after saving")
    
    args = parser.parse_args()
    
    merge_point_clouds(
        ply_files=args.files,
        output_file=args.output,
        voxel_size=args.voxel,
        visualize=args.visualize
    )


if __name__ == "__main__":
    main()
