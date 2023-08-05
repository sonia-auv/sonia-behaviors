from flexbe_core import EventState, Logger

import rospy
from sonia_common.msg import MultiAddPose, AddPose

class move_to_ctr(EventState):
    def __init__(self):
        super().__init__(outcomes=["success"],
                         input_keys=["obj_ctr"],
                         output_keys=["init_traj"])
        self.__move_pub = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)

    def execute(self, userdata):
        pose = AddPose()
        pose.frame = 1
        if abs(300-userdata.obj_ctr[0]) > 25:
            pose.position.y = (300-userdata.obj_ctr[0])*0.004
            Logger.loghint(f"Move in y: {pose.position.y}")
        

        userdata.init_traj = MultiAddPose()
        userdata.init_traj.pose.append(pose)
        return "success"

    # def on_exit(self, userdata):
    #     self.__move_pub.unregister()