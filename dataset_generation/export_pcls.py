import bpy
import numpy as np
import pathlib
import sys

OUTPUT_PATH = "/path/to/output"

def vec2arr(vec):
    return np.array([vec[0], vec[1], vec[2]])

def transform_pose(pose, R):
    transformed = R @ pose
    return vec2arr(transformed)

def export_pcl(R, obj, output_file):
    coords = np.array([transform_pose(v.co, R) for v in obj.data.vertices])
    connector_coords = np.array([transform_pose(v.co, R) for v in bpy.data.objects["connector"].data.vertices]).reshape(-1,3)
    outlier_coords = np.array([transform_pose(v.co, R) for v in bpy.data.objects["none"].data.vertices])

    pcl = np.r_[coords, connector_coords, outlier_coords]

    np.save(output_file, pcl)

if __name__ == "__main__":
    sample = pathlib.Path(bpy.context.blend_data.filepath).stem

    output_path = pathlib.Path(OUTPUT_PATH) / sample
    output_path.mkdir(exist_ok=True)

    pcl_path = output_path / "pcl"
    pcl_path.mkdir(exist_ok=True)

    for frame in range(1, 11):

        bpy.context.scene.frame_set(frame)

        reserved_obs = ["connector", "none"]

        main_ob = None
        connector_meshes = []
        for o in bpy.data.objects:
            if o.type == "MESH":
                for i, r in enumerate(reserved_obs):
                    if r in o.name:
                        break
                else:
                    main_ob = o

        bpy.context.view_layer.objects.active = main_ob
        bpy.data.objects[main_ob.name].select_set(True)
        depsgraph = bpy.context.evaluated_depsgraph_get()
        main_ob = bpy.context.active_object.evaluated_get(depsgraph)
        R_o = main_ob.matrix_world
        
        # print progress
        sys.stdout.write('\r')
        sys.stdout.write("[{: <70}] {:04d}/{:04d}".format('='*int(70*frame / 300), frame, 300))
        sys.stdout.flush()
        
        export_pcl(R_o, main_ob, pcl_path / "{:03d}".format(frame - 1))