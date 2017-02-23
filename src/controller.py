#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench
from dynamic_reconfigure.server import Server
from mod_ctrl_quad.cfg import ForcesConfig


class Controller:
    def __init__(self):
        self.wrench_pub = rospy.Publisher("/quad_dynamics/quad_wrench",Wrench,queue_size=1)
        self.forces_server = Server(ForcesConfig,self.callback)

    def callback(self,config,level):
        msg = Wrench()
        msg.force.z = config.F
        msg.torque.x = config.Tx
        msg.torque.y = config.Ty
        msg.torque.z = config.Tz
        self.wrench_pub.publish(msg)
        return config


if __name__ == "__main__":
    rospy.init_node("quad_controller")
    quad_control = Controller()

    rospy.spin()
