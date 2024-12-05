import bpy
from pathlib import Path
import numpy as np 
from mathutils import Vector
from scipy.spatial import distance_matrix

"""
This file needs to be run inside of Blender
Copy it into the scripting section
Run it once without changing anything - this should add the pointcloud from clean/000.ply for example and emptys for the grid
Run it again, but set the start node to one at the end of a wire - this will create the armature
"""

ROOT = Path('path/to/data')
FILE_INDEX = 0
start_node = None

def empty_at(pos, index):
    o = bpy.data.objects.new( "empty.{:04d}".format(index), None )
    o.location = pos
    o.empty_display_size = 0.005
    o.empty_display_type = 'PLAIN_AXES'   
    bpy.context.scene.collection.objects.link( o )

def get_outgoing_nodes(adjacency, node_id):
    outgoing_nodes = np.where(adjacency[node_id, :] == 1)[0]
    adjacency[outgoing_nodes, node_id] = 0
    adjacency[node_id, outgoing_nodes] = 0
    
    return outgoing_nodes

def get_closest_node(nodes, done, todo):
    dists = distance_matrix([nodes[v] for v in done],[nodes[v] for v in todo])
    indices = np.unravel_index(np.argmin(dists), dists.shape)
    return done[indices[0]], todo[indices[1]]

# bpy.ops.object.mode_set(mode='OBJECT')

bpy.ops.object.select_all()
bpy.ops.object.delete()
bpy.ops.object.select_all()
bpy.ops.object.delete()

graph = np.load(ROOT / 'skeleton' / '{:03d}.npz'.format(FILE_INDEX))
nodes = graph['nodes']
adj = graph['adj']

inds = np.where(adj == 1)
adj[inds[1], inds[0]] = 1

if start_node is None:
    for n, node in enumerate(nodes):
        empty_at(node, n)

else:

    # add the armature
    bpy.ops.object.armature_add()
    armature = bpy.data.armatures[-1]

    # clear the first pre existing bone
    bpy.ops.object.mode_set(mode='EDIT',toggle=True)
    for bone in armature.edit_bones:
        armature.edit_bones.remove(bone)

    open_list = [start_node]
    done_list = [start_node]

    while len(done_list) != len(nodes):
        if len(open_list) == 0:
            from_node, to_node = get_closest_node(nodes, done_list, [i for i in range(len(nodes)) if i not in done_list])
            open_list = [to_node]
            eb = armature.edit_bones.new("Bone.{:03d}".format(to_node))
            done_list.append(to_node)
            eb.head = nodes[from_node]
            eb.tail = nodes[to_node]
            
            eb.parent = armature.edit_bones["Bone.{:03d}".format(from_node)]
            eb.use_connect = True

        node_id = open_list.pop()
        
        outgoing_nodes = get_outgoing_nodes(adj, node_id)
        
        open_list += [n for n in outgoing_nodes if n not in done_list]
        
        for child_id in outgoing_nodes:
            if node_id == child_id:
                print("Warning - skipping edge from node {} to itself".format(node_id))
                continue
            if child_id in done_list:
                print("Warning - graph contains a circle at node {}!".format(child_id))
                eb = armature.edit_bones.new("Circle".format(child_id))
            else:
                eb = armature.edit_bones.new("Bone.{:03d}".format(child_id))
                done_list.append(child_id)
            eb.head = nodes[node_id]
            eb.tail = nodes[child_id]
            if node_id != start_node:
                eb.parent = armature.edit_bones["Bone.{:03d}".format(node_id)]
                eb.use_connect = True
                
    bpy.ops.object.mode_set(mode='OBJECT')
    
    bpy.ops.wm.ply_import(filepath=str((ROOT / 'clean' / '{:03d}.ply'.format(FILE_INDEX)).resolve()))