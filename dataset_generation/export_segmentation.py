import bpy
import numpy as np
import pathlib


"""
This file needs to be run inside of blender
All noise points should be in a group 'none'
All connector points should be in a group 'connectors'
The bifurcation and endpoint segmentation is done automatically
"""

OUTPUT_PATH = "/output/folder"

BIFURCATION_LAMBDA = 0.1
ENDPOINT_LAMBDA = 0.3
THICKNESS_SCALING = 100
UPPER_BOUND = 0.1
LOWER_BOUND = 0.01

def vec2arr(vec):
    return np.array([vec[0], vec[1], vec[2]])

def transform_pose(pose, R):
    transformed = R @ pose
    return vec2arr(transformed)

def estimate_bone_thickness(R_a, R_o, armature, ob, bone_id, group_lookup, connector_bones):
    bone = armature.pose.bones[bone_id]

    head_coord = vec2arr(R_a @ bone.head)
    tail_coord = vec2arr(R_a @ bone.tail)

    bone_direction = tail_coord - head_coord
    bone_length = np.linalg.norm(bone_direction)
    bone_direction_norm = bone_direction / bone_length

    vert_dists = []
    
    t_min, t_max = 0, 0
    
    if bone_id in group_lookup and bone_id not in connector_bones:
        for v_id in group_lookup[bone_id]:
            v = ob.data.vertices[v_id]
            
            bone_coord = vec2arr(R_o @ v.co)
            
            t = np.dot(bone_coord - head_coord, bone_direction_norm)
            if t < t_min:
                t_min = t
            if t > t_max:
                t_max = t
            
            projected_point = head_coord + t * bone_direction_norm
            
            vert_dists.append(np.linalg.norm(bone_coord - projected_point))
            
        return vert_dists, bone_length, t_min, t_max
    
    else:
        return [], bone_length, 0, 0

"""
Get the bone length using the assigned points to account for wrong labelling.
"""
def get_bone_length(R_a, R_o, armature, ob, bone_id, group_lookup, connector_bones, include_t_min=False, include_t_max=False):
    # print("GETTING {}".format(bone_id))
    bone = armature.pose.bones[bone_id]

    head_coord = vec2arr(R_a @ bone.head)
    tail_coord = vec2arr(R_a @ bone.tail)

    bone_direction = tail_coord - head_coord
    bone_length = np.linalg.norm(bone_direction)
    bone_direction_norm = bone_direction / bone_length

    t_min, t_max = 0, 0
    
    if include_t_min or include_t_max:
        if bone_id in group_lookup and bone_id not in connector_bones:
            for v_id in group_lookup[bone_id]:
                v = ob.data.vertices[v_id]
                
                bone_coord = vec2arr(R_o @ v.co)
                
                t = np.dot(bone_coord - head_coord, bone_direction_norm)
                if t < t_min:
                    t_min = t
                if t > t_max:
                    t_max = t
        else:
            print("WARNING: Bone {} not in group lookup".format(bone_id))

    if not include_t_max:
        t_max = bone_length
    if not include_t_min:
        t_min = 0

    return head_coord + t_min * bone_direction_norm, head_coord + t_max * bone_direction_norm, t_max - t_min

def segment_in_dir(R_a, R_o, armature, ob, start_bone, group_lookup, bone_lengths, seg_len, labels, label, is_start):
    current_bone = start_bone
    if is_start:
        i = 0
    else:
        i = -1
    
    while seg_len > bone_lengths[i]:
        if is_start and len(current_bone.children) != 1 or not is_start and current_bone.parent is None:
            break
        
        if current_bone.name in group_lookup:
            for v_id in group_lookup[current_bone.name]:
                labels[v_id] = label
        
        seg_len -= bone_lengths[i]
        if is_start:
            current_bone = current_bone.children[0]
            i += 1
        else:
            current_bone = current_bone.parent
            i -= 1
    
    # segment only a part of the last bonehead_coord = vec2arr(R_a @ bone.head)
    head_coord = vec2arr(R_a @ current_bone.head)
    tail_coord = vec2arr(R_a @ current_bone.tail)

    bone_direction = tail_coord - head_coord
    bone_length = bone_lengths[i]
    bone_direction_norm = bone_direction / bone_length
    
    if current_bone.name in group_lookup:
        for v_id in group_lookup[current_bone.name]:
            v = ob.data.vertices[v_id]
            vertex_coord = vec2arr(R_o @ v.co)
            t = np.dot(vertex_coord - head_coord, bone_direction_norm)
            
            if (is_start and t <= seg_len) or (not is_start and t > bone_length - seg_len):
                labels[v_id] = label

def thickness_fct(l, thickness):
    beta = UPPER_BOUND - LOWER_BOUND
    
    return l * (2 * beta * (1 - 1 / (1 + np.exp(-THICKNESS_SCALING * thickness))) + LOWER_BOUND)

def segment_from_bone(R_a, R_o, armature, ob, start_bone_id, is_end, group_lookup, labels, connector_bones):
    vert_dists, bone_length, t_min, t_max = estimate_bone_thickness(R_a, R_o, armature, ob, start_bone_id, group_lookup, connector_bones)
    
    start_label = 2
    start_lambda = BIFURCATION_LAMBDA
    start_overhead = 0
    
    if is_end:
        start_label = 1
        start_lambda = ENDPOINT_LAMBDA
        start_overhead = -t_min
    else:
        # print("Start intersection")
        pass
    
    bone_lengths = [bone_length]
    
    start_bone = armature.pose.bones[start_bone_id]
    current_bone = start_bone
    
    while len(current_bone.children) == 1:
        current_bone = current_bone.children[0]
        
        current_dists, bone_length, t_min, t_max = estimate_bone_thickness(R_a, R_o, armature, ob, current_bone.name, group_lookup, connector_bones)
        bone_lengths.append(bone_length)
        
        vert_dists += current_dists
    
    end_bone = current_bone
    end_label = 2
    end_lambda = BIFURCATION_LAMBDA
    end_overhead = 0
    
    if len(current_bone.children) == 0:
        end_label = 1
        end_lambda = ENDPOINT_LAMBDA
        end_overhead = t_max - bone_length
    else:
        # print("Found intersection")
        pass
    
    segment_length = sum(bone_lengths) + start_overhead + end_overhead
    print("From {} to {}: {}".format(start_bone_id, end_bone.name, segment_length))

    thickness = np.mean(vert_dists) + 3 * np.std(vert_dists)
    
    start_length = thickness_fct(start_lambda, thickness)
    end_length = thickness_fct(end_lambda, thickness)
    
    if start_label == 2:
        start_length += bone_lengths[0]
    if end_label == 2:
        end_length += bone_lengths[-1]
    
    start_length = min(start_length, start_lambda / (start_lambda + end_lambda) * segment_length) - start_overhead
    end_length = min(end_length, end_lambda / (start_lambda + end_lambda) * segment_length) - end_overhead
    
    print("Labeling start from bone {} length {} label {}".format(start_bone.name, start_length, start_label))
    if start_bone.name not in connector_bones:
        segment_in_dir(R_a, R_o, armature, ob, start_bone, group_lookup, bone_lengths, start_length, labels, start_label, is_start = True)
    
    print("Labeling end from bone {} length {} label {}".format(end_bone.name, end_length, end_label))
    if end_bone.name not in connector_bones:
        segment_in_dir(R_a, R_o, armature, ob, end_bone, group_lookup, bone_lengths, end_length, labels, end_label, is_start = False)
    
    for child in end_bone.children:
        segment_from_bone(R_a, R_o, armature, ob, child.name, False, group_lookup, labels, connector_bones)

def child_bones_wo_connectors(bone, connector_bones):
    return [c for c in bone.children if c.name not in connector_bones]

def get_segment_lengths(R_a, R_o, armature, ob, start_bone_id, group_lookup, connector_bones, is_start=True):

    current_seg = []
    current_seg_len = 0

    current_bone = armature.pose.bones[start_bone_id]

    while len(child_bones_wo_connectors(current_bone, connector_bones)) == 1:
        head, tail, bone_len = get_bone_length(R_a, R_o, armature, ob, current_bone.name, group_lookup, connector_bones, include_t_min=is_start)

        current_seg.append(head)
        current_seg_len += bone_len


        current_bone = child_bones_wo_connectors(current_bone, connector_bones)[0]
        is_start = False
    
    is_end_bone = len(child_bones_wo_connectors(current_bone, connector_bones)) == 0
    head, tail, bone_len = get_bone_length(R_a, R_o, armature, ob, current_bone.name, group_lookup, connector_bones, include_t_min=is_start, include_t_max=is_end_bone)

    current_seg.append(head)
    current_seg.append(tail)
    current_seg_len += bone_len

    segments = [current_seg]
    segment_lengths = [current_seg_len]

    for child in child_bones_wo_connectors(current_bone, connector_bones):
        child_segments, child_segment_lengths = get_segment_lengths(R_a, R_o, armature, ob, child.name, group_lookup, connector_bones, is_start=False)
        
        segments += child_segments
        segment_lengths += child_segment_lengths
    
    return segments, segment_lengths

def get_connector_bones(bones, ob):
    cb = set()
    for v_id, v in enumerate(ob.data.vertices):
        for g in v.groups:
            if g.group < len(ob.vertex_groups):
                bone_name = ob.vertex_groups[g.group].name
                cb.add(bone_name)

    return cb

def filter_connector_bones(connector_bones):
    def is_bone_valid(bone_id):
        if bone_id not in connector_bones:
            return False
        current_bone = armature.pose.bones[bone_id]
        return all([is_bone_valid(child.name) for child in current_bone.children])
    
    return [b for b in connector_bones if is_bone_valid(b)]

def build_group_lookup(obj):
    group_lookup = {}
    for v_id, v in enumerate(obj.data.vertices):
        for g in v.groups:
            if g.group < len(obj.vertex_groups):
                group_name = obj.vertex_groups[g.group].name
                # print(group_name)
                if group_name not in group_lookup:
                    group_lookup[group_name] = []
                group_lookup[group_name].append(v_id)
    return group_lookup

def export_labels(R, obj, labels, reserved_counts, output_file):
    for i, count in enumerate(reserved_counts):
        if count > 0:
            labels = np.r_[labels, np.full(count, 3 + i, dtype=int)]

    np.save(output_file, labels)

if __name__ == "__main__":
    sample = pathlib.Path(bpy.context.blend_data.filepath).stem

    output_path = pathlib.Path(OUTPUT_PATH)
    output_path.mkdir(exist_ok=True)

    armature = bpy.data.objects["Armature"]
    R_a = armature.matrix_world

    reserved_obs = ["connector", "none"]
    reserved_names = [[],[]]
    reserved_counts = [0,0]

    main_ob = None
    connector_meshes = []
    for o in bpy.data.objects:
        if o.type == "MESH":
            for i, r in enumerate(reserved_obs):
                if "connector" in o.name:
                    connector_meshes.append(o)
                if r in o.name:
                    reserved_counts[i] += len(o.data.vertices)
                    reserved_names[i].append(o.name)
                    break
            else:
                main_ob = o

    connector_bones = []
    for connector in connector_meshes:
        connector_bones += get_connector_bones(armature.pose.bones, connector)
    
    connector_bones = filter_connector_bones(connector_bones)

    bpy.context.view_layer.objects.active = main_ob
    bpy.data.objects[main_ob.name].select_set(True)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    main_ob = bpy.context.active_object.evaluated_get(depsgraph)
    R_o = main_ob.matrix_world

    bone_id = None
    for bone in armature.pose.bones:
        if bone.parent == None:
            if bone_id == None:
                bone_id = bone.name
            else:
                print("Object has two parent bones!")
                exit()

    group_lookup = build_group_lookup(main_ob)

    labels = np.zeros(len(main_ob.data.vertices), dtype=int)
    segment_from_bone(R_a, R_o, armature, main_ob, bone_id, True, group_lookup, labels, connector_bones)

    export_labels(R_o, main_ob, labels, reserved_counts, output_path / sample / "segmentation.npy")
