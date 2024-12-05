from pc_skeletor import skeletor
import numpy as np
import open3d as o3d
import glob
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.spatial import distance_matrix

input_files = sorted(glob.glob("./clean/*.ply"))

print("Files: ", input_files)

# Init tree skeletonizer
skeletor = skeletor.Skeletonizer(point_cloud=None,
                                     down_sample=0.0001,
                                     debug=False)

laplacian_config = {"MAX_LAPLACE_CONTRACTION_WEIGHT": 1024*0.5,
                    "MAX_POSITIONAL_WEIGHT": 1024*2,
                    "INIT_LAPLACIAN_SCALE": 10}

for idx, file in enumerate(input_files):
    print("Reading: ", file)
    pcd = o3d.io.read_point_cloud(file)

    # print number of points
    print("Number of points: ", np.asarray(pcd.points).shape[0])

    # extract the skeleton points using Laplacian contraction
    skeletor.pcd = pcd
    sceleton = skeletor.extract(method='Laplacian', config=laplacian_config)

    skeleton = np.asarray(sceleton[1].points)

    # calculate the distances between all the points
    C = distance_matrix(skeleton, skeleton)

    # calculate the adjacency matrix and convert it to binary
    mst = minimum_spanning_tree(C).toarray()
    mst = mst > 0

    # convert the directed graph to an undirected graph
    mst = np.maximum(mst, mst.T)

    # save to a npz file
    np.savez_compressed(f"{idx}.npz", nodes=skeleton, adj=mst)


