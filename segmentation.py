# -*- coding: future_fstrings -*-
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

colorcet = "tab20"
colorcet2 = "tab10"

def cluster(pcd):
    # 3d shape detection with RANSAC - (for plane outliers (in m), no. of pts to define plane, iterations)
    # clustering with DBSCAN - (epsilon = radius for cluster (in m), min pts in cluster)

    # RANSAC with Euclidean clustering
    segment_models = {} # plane parameters
    segments = {} # planar regions in PCD
    max_plane_idx = 20 # find 20 planes 
    rest = pcd
    dt = 0.1

    for i in range(max_plane_idx):
        colors = plt.get_cmap(colorcet)(i)
        
        segment_models[i], inliers = rest.segment_plane(distance_threshold = dt,ransac_n = 3,num_iterations = 1000) # RANSAC for best plane
        segments[i] = rest.select_down_sample(inliers)
        
        # use DBSCAN on RANSAC plane
        labels = np.array(segments[i].cluster_dbscan(eps = dt*10, min_points = 10)) # DBSCAN (dt*10 was author's choice, no scientific reason)
        candidates = [len(np.where(labels == j)[0]) for j in np.unique(labels)] # how many points each cluster holds

        best_candidate = int(np.unique(labels)[np.where(candidates == np.max(candidates))[0]]) # cluster which holds most points
        #TypeError: only size-1 arrays can be converted to Python scalars

        rest = rest.select_down_sample(inliers, invert=True) + segments[i].select_down_sample(list(np.where(labels != best_candidate)[0]))
        # selects outliers and points not belonging to best cluster

        segments[i] = segments[i].select_down_sample(list(np.where(labels == best_candidate)[0])) # index of points belonging to biggest cluster
        segments[i].paint_uniform_color(list(colors[:3]))
        print(f"best candidate = {best_candidate} (pass {i+1}/{max_plane_idx} done)")

    # Euclidean clustering of the rest with DBSCAN
    labels = np.array(rest.cluster_dbscan(eps = 0.05, min_points = 5)) #varies from 0 to n (cluster labels) (-1 is noise)
    max_label = labels.max()
    print(f"point cloud has {max_label+1} clusters")

    colors = plt.get_cmap(colorcet2)(labels / max(max_label, 1)) # 1 when max_label is noise
    colors[labels < 0] = 0 # noise is black in colour
    rest.colors = o3d.utility.Vector3dVector(colors[:, :3]) # attributes for R, G, B

    final = [segments[i] for i in range(max_plane_idx)]+[rest]
    for pc in final:
        print(pc.get_geometry_type())
    o3d.visualization.draw_geometries(final)
    return final