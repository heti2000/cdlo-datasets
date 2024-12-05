import bpy

"""
Add inverse kinematics and controls to the object
"""

print("---Adding inverse kinematics and controls---")

def search_bone(T, ob, bone):
    if bone.parent is not None:
        setchild = bone.constraints.new(type='CHILD_OF')
        
        setchild.target = ob
        setchild.subtarget = bone.parent.name
    
    bone_count = 1
    while len(bone.children) == 1:
        bone_count += 1
        bone = bone.children[0]

    control = bpy.data.objects.new( "Control", None )

    control.empty_display_type = 'PLAIN_AXES'
    control.location = T @ bone.tail
    control.rotation_euler = bone.matrix.to_euler()
    control.scale = (0.05,0.05,0.05)

    # due to the new mechanism of "collection"
    bpy.context.scene.collection.objects.link( control )
    
    # add inverse kinematics to the model
    constraint = bone.constraints.new(type='IK')
    
    constraint.target = control
    constraint.use_rotation = True
    constraint.chain_count = bone_count
    
    for b in bone.children:
        search_bone(T, ob, b)

for ob in bpy.data.objects:
    
    if ob.type == 'ARMATURE':
        
        armature = ob
        T = ob.matrix_world
        break

for bone in ob.pose.bones:
    if bone.parent == None:
        break

search_bone(T, ob, bone)
                

print("---DONE---")