import bpy
import numpy as np
import pathlib
import sys

OUTPUT_PATH = "/output/path"

def vec2arr(vec):
    return np.array([vec[0], vec[1], vec[2]])

def transform_pose(pose, R):
    transformed = R @ pose
    return vec2arr(transformed)

def extract_skeleton(R_a, current_bone, current_bone_id, coords, adjacency, return_next_bone_id=False):
    if current_bone_id == 0:
        coords[0] = transform_pose(current_bone.head, R_a)
        adjacency[0,1] = 1
        adjacency[1,0] = 1
        current_bone_id += 1
    
    coords[current_bone_id] = transform_pose(current_bone.tail, R_a)
    
    next_bone_id = current_bone_id + 1
    for child in current_bone.children:
        adjacency[current_bone_id, next_bone_id] = 1
        adjacency[next_bone_id, current_bone_id] = 1
        next_bone_id = extract_skeleton(R_a, child, next_bone_id, coords, adjacency, return_next_bone_id=True)

    if return_next_bone_id:
        return next_bone_id

if __name__ == "__main__":
    sample = pathlib.Path(bpy.context.blend_data.filepath).stem

    output_path = pathlib.Path(OUTPUT_PATH)
    output_path.mkdir(exist_ok=True)

    skeleton_path = output_path / sample / "skeletons"
    skeleton_path.mkdir(exist_ok=True, parents=True)

    for frame in range(1, 11):

        bpy.context.scene.frame_set(frame)

        armature = bpy.data.objects["Armature"]
        R_a = armature.matrix_world

        bone_id = None
        for bone in armature.pose.bones:
            if bone.parent == None:
                if bone_id == None:
                    bone_id = bone.name
                else:
                    print("Object has two parent bones!")
                    exit()
        
        # print progress
        sys.stdout.write('\r')
        sys.stdout.write("[{: <70}] {:04d}/{:04d}".format('='*int(70*frame / 300), frame, 300))
        sys.stdout.flush()

        num_bones = len(armature.pose.bones)
        adj = np.zeros((num_bones+1, num_bones+1), dtype=bool)
        coords = np.zeros((num_bones+1, 3))

        extract_skeleton(R_a, armature.pose.bones[bone_id], 0, coords, adj)

        np.savez(skeleton_path / "{:03d}.npz".format(frame), nodes=coords, adj=adj)