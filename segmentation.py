# -*- coding: future_fstrings -*-
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

cmap = "tab10"
cmap2 = "tab10"

def cluster(pcd):
    # 3d shape detection with RANSAC - (for plane outliers (in m), no. of pts to define plane, iterations)
    # clustering with DBSCAN - (epsilon = radius for cluster (in m), min pts in cluster)

    # filtering data to remove sparse objects (far away objects)
    max_dist = 5.0
    min_dist = 0.6
    points = np.asarray(pcd.points)
    pcd = pcd.select_down_sample(np.where(np.logical_and(
            min_dist < (points[:,0]*points[:,0] + points[:,1]*points[:,1] + points[:,2]*points[:,2]),
            (points[:,0]*points[:,0] + points[:,1]*points[:,1] + points[:,2]*points[:,2]) < max_dist))[0])

    # RANSAC with Euclidean clustering
    segment_models = {} # plane parameters
    segments = {} # planar regions in PCD
    max_plane_idx = 8 # find n planes 
    rest = pcd
    dt = 0.04 # distance threshold for RANSAC
    fac = 5 # factor for DBSCAN clustering
    min_pts = 10 # min points in a cluster

    for i in range(max_plane_idx):
        colors = plt.get_cmap(cmap)(i)
        
        segment_models[i], inliers = rest.segment_plane(distance_threshold = dt, ransac_n = 3, num_iterations = 1000) # RANSAC for best plane
        segments[i] = rest.select_down_sample(inliers)
        
        # use DBSCAN on RANSAC plane
        labels = np.array(segments[i].cluster_dbscan(eps = dt*fac, min_points = min_pts)) # DBSCAN
        candidates = [len(np.where(labels == j)[0]) for j in np.unique(labels)] # how many points each cluster holds
        
        if(len(candidates) == 0):
            i=i-1
            continue
        best_candidate = int(np.unique(labels)[np.where(candidates == np.max(candidates))[0]]) # cluster which holds most points

        rest = rest.select_down_sample(inliers, invert=True) + segments[i].select_down_sample(list(np.where(labels != best_candidate)[0]))
        # selects outliers and points not belonging to best cluster

        segments[i] = segments[i].select_down_sample(list(np.where(labels == best_candidate)[0])) # index of points belonging to biggest cluster
        segments[i].paint_uniform_color(list(colors[:3]))
        print(f"best candidate = {best_candidate} (pass {i+1}/{max_plane_idx} done)")

    # Euclidean clustering of the rest with DBSCAN
    labels = np.array(rest.cluster_dbscan(eps = 0.5, min_points = 10)) #varies from 0 to n (cluster labels) (-1 is noise)
    max_label = labels.max()
    max_label = max(1, max_label)
    
    colors = plt.get_cmap(cmap2)(labels / max_label) # 1 when max_label is noise
    colors[labels < 0] = 0 # noise is black in colour
    rest.colors = o3d.utility.Vector3dVector(colors[:, :3]) # attributes for R,G,B

    final = [segments[i] for i in range(max_plane_idx)]+[rest]

    # temp = [final[0], final[1]]
    # o3d.visualization.draw_geometries(final)
    return final