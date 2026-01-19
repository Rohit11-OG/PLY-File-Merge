#!/usr/bin/env python3
"""
Advanced Multi-way PLY Registration and Merger
----------------------------------------------
Aligns and merges multiple PLY point clouds (e.g., different angles of a tank)
using Global Registration and Pose Graph Optimization.

Usage:
    python register_and_merge.py *.ply --visualize
"""

import open3d as o3d
import numpy as np
import argparse
import sys
import os
import copy
import json

def preprocess_point_cloud(pcd, voxel_size):
    """
    Downsample, estimate normals, and compute FPFH features.
    """
    # Downsample
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # Estimate normals (required for ICP and feature computation)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    # Compute FPFH features
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def pairwise_registration(source, target, voxel_size):
    """
    Perform pairwise registration:
    1. Global RANSAC registration (on heavily downsampled cloud)
    2. ICP registration (on finer cloud)
    """
    
    # 1. Coarse Global Registration
    # Downsample heavily (aim for ~5000 points or voxel_size * 5)
    # This speeds up RANSAC significantly
    voxel_size_coarse = voxel_size * 5
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size_coarse)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size_coarse)
    
    result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size_coarse)
    
    # 2. Local Refinement (Point-to-Plane ICP)
    # Uses the finer voxel size (passed in args)
    distance_threshold = voxel_size * 1.5
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    
    return result_icp.transformation, result_icp

def full_registration(pcds, voxel_size):
    """
    Build a Pose Graph and optimize it for global consistency.
    """
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    
    n_pcds = len(pcds)
    
    # Preprocess all point clouds
    print(f"Preprocessing {n_pcds} point clouds (voxel_size={voxel_size:.4f})...")
    pcds_down = []
    pcds_fpfh = []
    for i, pcd in enumerate(pcds):
        pcd_d, pcd_f = preprocess_point_cloud(pcd, voxel_size)
        pcds_down.append(pcd_d)
        pcds_fpfh.append(pcd_f)
        print(f"  Processed cloud {i}: {len(pcd_d.points)} points (downsampled)")

    print("\nRunning Pairwise Registration...")
    # Matches adjacent pairs efficiently (assuming sequential capture)
    # For unstructured data, you might want to match all pairs
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            # Only align if they are close in sequence (e.g., i and i+1)
            # You can increase this range if files are not strictly sequential
            if target_id == source_id + 1: 
                print(f"  Aligning Pair {source_id} -> {target_id}...")
                
                # Note: We pass the 'downsampled' clouds for ICP speed, 
                # but pairwise_registration will downsample logic internally for RANSAC
                trans, info = pairwise_registration(
                    pcds_down[source_id], pcds_down[target_id],
                    voxel_size
                )
                
                # Compute Information Matrix (required for Pose Graph)
                information_matrix = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    pcds_down[source_id], pcds_down[target_id],
                    voxel_size * 1.5, trans)

                if target_id == source_id + 1:
                    odometry = np.dot(trans, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                    
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            source_id, target_id, trans,
                            information_matrix, uncertain=False))
            
            # Loop Closure Detection (Non-adjacent pairs)
            # Check every pair (or a subset) to see if they match well
            # This is critical for 360 scans to "close the circle"
            elif source_id % 2 == 0: # Check spare pairs to save time, or check all for robustness
                 print(f"  Checking Loop Closure {source_id} -> {target_id}...")
                 trans, info = pairwise_registration(
                    pcds_down[source_id], pcds_down[target_id],
                    voxel_size
                 )
                 
                 # Only add edge if the registration is confident (good RMSE/Fitness)
                 if info.fitness > 0.1 and info.inlier_rmse < voxel_size * 2:
                    print(f"    -> Loop Closure Found! (Fitness: {info.fitness:.2f})")
                    information_matrix = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                        pcds_down[source_id], pcds_down[target_id],
                        voxel_size * 1.5, trans)
                    
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            source_id, target_id, trans,
                            information_matrix, uncertain=True))
            
    print("\nrunning Pose Graph Optimization...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 1.5,
        edge_prune_threshold=0.25,
        reference_node=0)
    
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
        
    return pose_graph

def main():
    parser = argparse.ArgumentParser(description="Multi-way registration of PLY files")
    parser.add_argument("files", nargs="+", help="List of PLY files to merge")
    parser.add_argument("--voxel", type=float, default=0.005, help="Voxel size (unitless, e.g., 0.005 meters)")
    parser.add_argument("--visualize", action="store_true", help="Visualize result")
    parser.add_argument("-o", "--output", default="merged_tank_aligned.ply", help="Output file")
    
    args = parser.parse_args()
    
    if len(args.files) < 2:
        print("Need at least 2 files to register!")
        sys.exit(1)
        
    # Load Files
    pcds = []
    print("Loading files...")
    for f in args.files:
        pcd = o3d.io.read_point_cloud(f)
        pcds.append(pcd)
        print(f"  Loaded {os.path.basename(f)}: {len(pcd.points)} points")
        
    # 1. Run Multi-way Registration
    print("\nStarting Registration Pipeline...")
    pose_graph = full_registration(pcds, args.voxel)
    
    # 2. Transform original clouds and merge
    print("\nMerging aligned clouds and extracting orientations...")
    merged_pcd = o3d.geometry.PointCloud()
    poses_data = {}
    visual_elements = []  # List to store coordinate frames for visualization

    for i, pcd in enumerate(pcds):
        # Apply the optimized transformation to the original high-res cloud
        # Note: Pose graph nodes are stored as Inverse of the pose
        trans = pose_graph.nodes[i].pose
        inv_trans = np.linalg.inv(trans)
        
        # Save pose info
        poses_data[os.path.basename(args.files[i])] = inv_trans.tolist()
        
        # Add visual coordinate frame at the camera position
        # Size 0.1 means 10cm axes (adjust based on your scale)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        axis.transform(inv_trans)
        visual_elements.append(axis)

        pcd_transformed = copy.deepcopy(pcd)
        pcd_transformed.transform(inv_trans)
        merged_pcd += pcd_transformed
    
    # Save poses to JSON
    with open("poses.json", "w") as f:
        json.dump(poses_data, f, indent=4)
    print("Saved camera poses to poses.json")
        
    # Post-processing: Uniform Density
    print(f"Applying final uniformity check (voxel_size={args.voxel})...")
    merged_pcd = merged_pcd.voxel_down_sample(voxel_size=args.voxel)
    
    # Recenter to (0,0,0)
    print("Recentering object to (0,0,0)...")
    center = merged_pcd.get_center()
    merged_pcd.translate(-center)
    # Also adjust visual elements
    for elem in visual_elements:
        elem.translate(-center)

    # 3. Save
    print(f"Saving {len(merged_pcd.points)} points to {args.output}...")
    o3d.io.write_point_cloud(args.output, merged_pcd)
    print("Done!")
    
    # 4. Visualize
    if args.visualize:
        print("Visualizing (Red/Green/Blue axes show camera positions)...")
        merged_pcd.estimate_normals() # Looks better with lighting
        visual_elements.append(merged_pcd)
        o3d.visualization.draw_geometries(visual_elements, 
            window_name="Aligned & Merged Tank (with Orientations)", 
            width=1024, height=768)

if __name__ == "__main__":
    main()
