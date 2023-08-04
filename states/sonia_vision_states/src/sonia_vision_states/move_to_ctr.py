from flexbe_core import EventState

import rospy
from sonia_common.msg import MultiAddPose, AddPose
class move_to_ctr(EventState):
    def __init__(self):
        super().__init__(outcomes=["success"],
                         input_keys=["obj_ctr"])
        self.__move_pub = None

    def on_enter(self, userdata):
        self.__move_pub = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)

    def execute(self, userdata):
        pose = AddPose()
        pose.frame = 1
        if abs(300-userdata.obj_ctr.x) > 25:
            pose.position.y = (300-userdata.obj_ctr.x*0.007)
        
        if abs(300-userdata.obj_ctr.y) > 25:
            pose.position.z= (200-userdata.obj_ctr.y*0.007)

        poses = MultiAddPose()
        poses.pose.append(pose)
        self.__move_pub.publish(poses)
        return "success"

    def on_exit(self, userdata):
        self.__move_pub.unregister()