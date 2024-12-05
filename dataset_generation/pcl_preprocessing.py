import open3d as o3d
import numpy as np
import itertools
import sys
import glob
from tqdm import tqdm

Z_THRESH_LOW = 0.000142*3 #0.00025
Z_THRESH_HIGH = 0.1
STATISTICAL_REMOVAL_NB_NEIGHBOURS = 30
STATISTICAL_REMOVAL_STD_RATIO = 2.0

def rot_matrix_from_normals(current_normal: np.array, new_normal: np.array):
    v = np.cross(current_normal, new_normal)
    cross_mat = np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0]
    ])

    cosine = np.dot(current_normal, new_normal)

    return np.eye(3) + cross_mat + np.matmul(cross_mat, cross_mat) / (1 + cosine)

def crop_z(pcd, low, high):
    bounds = [[-np.inf, np.inf], [-np.inf, np.inf], [low, high]]  # set the bounds
    bounding_box_points = list(itertools.product(*bounds))  # create limit points
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    # Crop the point cloud using the bounding box:
    return pcd.crop(bounding_box)

def main(argv):
    input_ply = argv[1] if len(argv) > 1 else "in.ply"
    output_ply = argv[2] if len(argv) > 2 else "out.ply"

    input_files = glob.glob(input_ply)

    for idx, file in enumerate(tqdm(input_files)):

        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        mesh = mesh.scale(0.05, center=[0, 0, 0])

        # scale the pointcloud to meters
        pcd = o3d.io.read_point_cloud(file)

        # rotate the cloud to be parallel to the ground
        [a, b, c, d], plane_ind = pcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=10000)
        normal = [a, b, c]
        normal /= np.linalg.norm(normal)
        pcd = pcd.rotate(rot_matrix_from_normals(normal, [0, 0, -1]), center=(0, 0, 0))

        plane = o3d.geometry.PointCloud()
        plane = pcd.select_by_index(plane_ind)

        # move the groundplane to zero
        # the plane detection is done again, because otherwise the error on the z axis would be too high
        [a, b, c, d], plane_ind = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
        pcd = pcd.translate((0, 0, d))

        # crop the z axis to remove the ground and things above
        pcd = crop_z(pcd, Z_THRESH_LOW, Z_THRESH_HIGH)

        # remove outliers
        _, ind = pcd.remove_statistical_outlier(
            nb_neighbors=STATISTICAL_REMOVAL_NB_NEIGHBOURS,
            std_ratio=STATISTICAL_REMOVAL_STD_RATIO
        )

        outlier_cloud = pcd.select_by_index(ind, invert=True)
        outlier_cloud.paint_uniform_color([1, 0, 0])
        pcd = pcd.select_by_index(ind)
        # center pointcloud along x, y axis
        [dx, dy, _] = pcd.get_center()
        pcd = pcd.translate((-dx, -dy, 0))
        pcd.paint_uniform_color([0.8, 0.8, 0.8])
        outlier_cloud = outlier_cloud.translate((-dx, -dy, 0))

        # o3d.visualization.draw_geometries([mesh, pcd, outlier_cloud])
        output_ply = argv[2]
        output_ply += "{:03d}".format(idx) + ".ply"
        # print("Saving to '{}'".format(output_ply))
        o3d.io.write_point_cloud(output_ply, pcd)


if __name__ == "__main__":
    main(sys.argv)
