#!/usr/bin/env python  
# Copyright (c) 2014 Hanson Robotics, Ltd. 
#import roslib; roslib.load_manifest('dmitry_tracker')
import rospy
import bpy
import math
import threading

from mathutils import Matrix, Vector
from math import acos, degrees

from bpy.app.handlers import persistent
from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray

from ros_pololu_servo.msg import servo_pololu

doeyes = True
domotors = False
leye = 0
reye = 0
jaw = 0
neck0 = 0
neck1 = 0
neck2 = 0
neck3 = 0

namespace = rospy.get_namespace()
rospy.init_node('blender_arm', anonymous=True)

if doeyes:
    pololu = rospy.Publisher(namespace + 'cmd_pololu', servo_pololu)

if domotors: 
    jawdm = rospy.Publisher(namespace + 'jaw/command', Float64)
    neck0dm = rospy.Publisher(namespace + 'neck0/command', Float64)
    neck1dm = rospy.Publisher(namespace + 'neck1/command', Float64)
    neck2dm = rospy.Publisher(namespace + 'neck2/command', Float64)
    neck3dm = rospy.Publisher(namespace + 'neck3/command', Float64)

def faceCallback(thearray):
    facemarker = bpy.data.objects['facedetect']
    facemarker.location.x = 1.5 - (thearray.data[0] / 100) 
    facemarker.location.z = 1.5 - (thearray.data[1] / 100)
    print("CALLBACK %s" % thearray)

facedetect = rospy.Subscriber('/facedetect', UInt16MultiArray, faceCallback)

def get_pose_matrix_in_other_space(mat, pose_bone):
    """ Returns the transform matrix relative to pose_bone's current
    transform space. In other words, presuming that mat is in
    armature space, slapping the returned matrix onto pose_bone
    should give it the armature-space transforms of mat.
    TODO: try to handle cases with axis-scaled parents better.
    """
    rest = pose_bone.bone.matrix_local.copy()
    rest_inv = rest.inverted()
    if pose_bone.parent:
        par_mat = pose_bone.parent.matrix.copy()
        par_inv = par_mat.inverted()
        par_rest = pose_bone.parent.bone.matrix_local.copy()
    else:
        par_mat = Matrix()
        par_inv = Matrix()
        par_rest = Matrix()

    # Get matrix in bone's current transform space
    smat = rest_inv * (par_rest * (par_inv * mat))

    # Compensate for non-local location
    #if not pose_bone.bone.use_local_location:
    # loc = smat.to_translation() * (par_rest.inverted() * rest).to_quaternion()
    # smat.translation = loc

    return smat

def get_local_pose_matrix(pose_bone):
    """ Returns the local transform matrix of the given pose bone.
    """
    return get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)

def get_bones_rotation(armature,bone,axis):
    mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
    if axis == 'z':
        return degrees(mat.to_euler().z)
    elif axis == 'y':
        return degrees(mat.to_euler().y)
    elif axis == 'x':
        return degrees(mat.to_euler().x)

def get_bones_rotation_rad(armature,bone,axis):
    mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
    if axis == 'z':
        return mat.to_euler().z
    elif axis == 'y':
        return mat.to_euler().y
    elif axis == 'x':
        return mat.to_euler().x

@persistent
def load_handler(dummy):
    global leye, reye, pololu, doeyes, domotors, jaw, neck0, neck1, neck2, neck3, jawdm, neck0dm, neck1dm, neck2dm, neck3dm

    newstuff = False

    newleye = get_bones_rotation_rad('leye','leyebone','z')
    if leye != newleye:
        leye = newleye
        eyedeg = get_bones_rotation('leye','leyebone','z')
        if doeyes: 
            msg = servo_pololu()
            msg.id = 0
            msg.angle = newleye
            msg.speed = 0
            msg.acceleration = 0
            pololu.publish(msg)
            print("LEYE: %s" % msg)

    newreye = get_bones_rotation_rad('reye','reyebone','z')
    if reye != newreye:
        reye = newreye
        eyedeg = get_bones_rotation('reye','reyebone','z')
        if doeyes: 
            msg = servo_pololu()
            msg.id = 1
            msg.angle = newreye
            msg.speed = 0
            msg.acceleration = 0
            pololu.publish(msg)
            print("REYE: %s" % msg)

    newjaw = get_bones_rotation_rad('jawarm','jawbone','x')
    if jaw != newjaw:
        jaw = newjaw
        jawdeg = get_bones_rotation('jawarm','jawbone','x')
        if domotors: jawdm.publish(float(jaw))
        print("JAW: %s (%s)" % (jaw, jawdeg))

    newneck0 = get_bones_rotation_rad('Armature','base','y') + 2.2
    if neck0 != newneck0:
        neck0 = newneck0
        neck0deg = get_bones_rotation('Armature','base','y')
        if domotors: neck0dm.publish(float(newneck0))
        print("BASE: %s (%s)" % (neck0, neck0deg))

    newneck1 = (get_bones_rotation_rad('Armature','bracket1','x') * -2) + 2.5
    if neck1 != newneck1:
        neck1 = newneck1
        if domotors: neck1dm.publish(float(newneck1))
        print("NECK1: %s" % neck1)

    newneck2 = (get_bones_rotation_rad('Armature','bracket2','z') * 2) + 2
    if neck2 != newneck2:
        neck2 = newneck2
        if domotors: neck2dm.publish(float(newneck2))
        print("NECK2: %s" % neck2)

    newneck3 = (get_bones_rotation_rad('Armature','bracket3','x') * 2) + 3
    if neck3 != newneck3:
        neck3 = newneck3
        if domotors: neck3dm.publish(float(newneck3))
        print("NECK3: %s" % neck3)

bpy.app.handlers.scene_update_post.append(load_handler)

print("Started")
