from flexbe_core import EventState, Logger
import rospy

from sonia_common.msg import BoundingBox2D, MultiAddPose, AddPose                                                                                                                                                                                                       

class CalculateMoveX(EventState):
    def __init__(self, target_area, step=0.3, num_steps=3, tolerance=0.1):
        super().__init__(outcomes=["success", "move"],
                         input_keys=["area"],
                         output_keys=["trajectory"])
        self.__target_area = target_area
        self.__step = step
        self.__num_steps = num_steps

    def execute(self, userdata):
        step_value = self.__target_area / self.__num_steps
        step_values = []
        for x in range(1, self.__num_steps+1):
            step_values.append(x*step_value)
        
        dist = None
        for x in step_values:
            if userdata.area < x:
                dist = x

        if dist is None:
            dist = self.__step
        
        userdata.trajectory = MultiAddPose()
        pose = AddPose()
        pose.frame = 1
        pose.position.x = self.__align_x(userdata.area)
        userdata.trajectory.pose.append(pose)
        if pose.position.y != 0 or pose.position.z != 0:
            return "move"
        return "success"
    
    def __align_x(self, area):
        area_offset = (area - self.__target_area)
        if abs(area_offset) > (self.__bounding_box.size_x/2):
            return 1 if area_offset > 0 else 0
        return 0
