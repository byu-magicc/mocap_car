#!/usr/bin/env python

#from __future__ import print_function

import argparse
import rospy
import math
import numpy as np

from car_autopilot.msg import State
from kb_utils.msg import Encoder
from geometry_msgs.msg import PoseStamped


class Mocap_To_State(object):

    def __init__(self, vehicle_name):
        """
        translate mocap into state
        """
        self.vel = 0;
        self.state_pub = rospy.Publisher("mocap", State, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/vrpn_client_node/{}/pose".format(vehicle_name), PoseStamped, self.pub_state)
        self.enc_sub = rospy.Subscriber("encoder", Encoder, self.update_encoder)

    def update_encoder(self, msg):
        self.vel = msg.vel

    def pub_state(self, msg):
        state_msg = State()
        state_msg.header = msg.header
        state_msg.p_north = msg.pose.position.z
        state_msg.p_east = -msg.pose.position.x
        w = msg.pose.orientation.w
        y = msg.pose.orientation.x
        z = msg.pose.orientation.y
        x = msg.pose.orientation.z

        # # define a quaternion to rotate about the x-axis by 90 degrees
        # w_r = math.cos(np.pi/2)
        # x_r = 0
        # y_r = math.sin(np.pi/2)
        # z_r = 0
        # # multiply them together to do the rotation
        # w_f = w_r*w - y_r*y
        # x_f = w_r*x + y_r*z
        # y_f = w_r*y + y_r*w
        # z_f = w_r*z - y_r*x
        state_msg.psi = -math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        # val = 2*(w*y - x*z)
        # if (math.fabs(val) > 1):
        #     state_msg.psi = np.sign(val)*np.pi/2;
        # else:
        #     state_msg.psi = math.asin(val);
        state_msg.u = self.vel
        self.state_pub.publish(state_msg)

    def clean_shutdown(self):
        print("\nExiting kb_driver...")
        return True

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


def main():
    """translates mocap data to car_autopilot/State"""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    parser.add_argument('vehicle_name', type=str, default='')
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("mocap_to_state", log_level=rospy.DEBUG)

    mocap_to_state = Mocap_To_State(args.vehicle_name)
    rospy.on_shutdown(mocap_to_state.clean_shutdown)
    mocap_to_state.run()

    print("Done.")

if __name__ == '__main__':
    main()
