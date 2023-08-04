from flexbe_core import EventState

import ros
from sonia_common.msg import MultiAddPose, AddPose

class pipe_align_angle(EventState):

    def __init__(self, max_attempts=10, tolerance=1.0):
        super().__init__(outcomes=["aligned", "moved", "failed"],
                         input_keys=["avg_angle", "counter"],
                         output_keys=["counter"])
        self.__max_attempts = max_attempts
        self.__tolerance = tolerance
        
    def on_enter(self, userdata):
        if len(userdata.counter) == 0:
            userdata.counter.append(0)
    
    def execute(self, userdata):
        if userdata.counter > self.__max_attempts:
            return "false"
        if abs(userdata.avg_angle) < self.__tolerance:
            return "aligned"
        pose = AddPose()
        pose.frame = 1
        pose.orientation.z = userdata.avg_angle
        poses = MultiAddPose()
        poses.pose.append(pose)
        userdata.counter += 1
        return "moved"
    
    