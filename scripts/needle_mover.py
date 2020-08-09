from __future__ import print_function
from ambf_client import Client
from PyKDL import Vector, Rotation
import time
import rospy
import obj_control_gui_needle as obj_control_gui
import math


def threshold(input):
    if abs(input) > 0.00001:
        return input
    else:
        return 0


def needle_mover(client, needle_name):
    # get handle from ambf
    print("getting handle from ambf")
    needle_handle = client.get_obj_handle(needle_name)

    # create gui object for needle
    needle_xyz = [0.0, 0.0, 0.0]
    needle_rpy = [0.0, 0.0, 0.0]
    OG_needle = obj_control_gui.ObjectGUI(
        needle_name, needle_xyz, needle_rpy, 2.0, 3.14, 0.000001)

    # loop until ros shutdown
    print("starting control loop")
    print("target pose:")
    while not rospy.is_shutdown():

        # Get current pose of needle
        Pos = Vector(needle_handle.get_pos().x,
                           needle_handle.get_pos().y,
                           needle_handle.get_pos().z)

        Rot = Rotation.RPY(needle_handle.get_rpy()[0],
                                needle_handle.get_rpy()[1],
                                needle_handle.get_rpy()[2])

        # update GUI for fresh values
        OG_needle.App.update()

        # values are floored to 0 if magnitude of slider is less than 0.00001
        delta_xyz = Vector(threshold(OG_needle.x*0.001*math.exp(abs(OG_needle.x)*2)), threshold(OG_needle.y*0.001*math.exp(abs(OG_needle.y)*2)), threshold(OG_needle.z*0.001*math.exp(abs(OG_needle.z)*2)))
        delta_rpy = Rotation.RPY(threshold(OG_needle.ro*0.001*math.exp(abs(OG_needle.ro)*2)), threshold(OG_needle.pi*0.001*math.exp(abs(OG_needle.pi)*2)), threshold(OG_needle.ya*0.001*math.exp(abs(OG_needle.ya)*2)))


        Pos=Pos + delta_xyz
        Rot=Rot*delta_rpy

        print([Pos]+[Rot.GetRPY()], end='\r')

        # skips if no input to avoid conflict with in-sim forces
        if delta_xyz.Norm() > 0.00001:
            needle_handle.set_pos(Pos[0], Pos[1], Pos[2])
        if abs(delta_rpy.GetRPY()[0]) > 0.00001 or abs(delta_rpy.GetRPY()[1]) > 0.00001 or abs(delta_rpy.GetRPY()[2]) > 0.00001:
            needle_handle.set_rot(Rot.GetQuaternion())

        time.sleep(0.005)



if __name__ == "__main__":
    c=Client('test')
    c.connect()
    needle_mover(c, 'Needle')
