from flexbe_core import EventState

from sonia_common.msg import MultiAddPose, AddPose

class DeltasToMultipose(EventState):

    def __init__(self):
        super().__init__(outcomes=["success"],
                         input_keys=["delta_x", "delta_y", "delta_z"],
                         output_keys=["trajectory"])
        
    def execute(self, userdata):
        pose = AddPose()
        pose.frame = 1
        pose.position.x = userdata.delta_x
        pose.position.y = userdata.delta_y
        pose.position.z = userdata.delta_z
        userdata.trajectory = MultiAddPose()
        userdata.trajectory.pose.append(pose)
        return "success"