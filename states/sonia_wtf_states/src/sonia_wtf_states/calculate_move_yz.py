from flexbe_core import EventState, Logger
import rospy

from sonia_common.msg import BoundingBox2D, MultiAddPose, AddPose                                                                                                                                                                                                       

class CalculateMoveYZ(EventState):
    def __init__(self, target_area, bounding_box_topic="box_target", step=0.3, num_steps=3):
        super().__init__(outcomes=["success", "move"],
                         input_keys=["delta_x", "delta_y", "area"],
                         output_keys=["trajectory"])
        self.__target_area = target_area
        self.__step = step
        self.__num_steps = num_steps
        self.__bounding_box_topic=bounding_box_topic
        self.__bounding_box_sub = None
        self.__bounding_box = None

    def on_enter(self, userdata):
        self.__bounding_box_sub = rospy.Subscriber("/proc_image_processing/" + self.__bounding_box_topic, BoundingBox2D, self.__bounding_box_sub_cb)
        
    def execute(self, userdata):
        step_value = self.__target_area / self.__num_steps
        step_values = []
        for x in range(1, self.__num_steps+1):
            step_values.append(x*step_value)
        
        if self.__bounding_box is None:
            Logger.logwarn("No Bounding Box Found. Defaulting to 50x50 at 0,0")
            self.__bounding_box = BoundingBox2D()
            self.__bounding_box.size_x = 50
            self.__bounding_box.size_y = 50
        
        dist = None
        for x in step_values:
            if userdata.area < x:
                dist = x

        if dist is None:
            dist = self.__step
        
        userdata.trajectory = MultiAddPose()
        pose = AddPose()
        pose.frame = 1
        
        pose.position.y = dist * self.__align_y(self, userdata.delta_x)
        pose.position.z = dist * self.__align_z(self, userdata.delta_x)
        userdata.trajectory.pose.append(pose)
        if pose.position.y != 0 or pose.position.z != 0:
            return "move"
        return "success"
    
    def __align_y(self, delta_x):
        x_offset = (delta_x - self.__bounding_box.center.x)
        if abs(x_offset) > (self.__bounding_box.size_x/2):
            return 1 if x_offset > 0 else -1
        return 0

    def __align_z(self, delta_y, pose):
        y_offset = (delta_y - self.__bounding_box.center.y)
        if abs(y_offset) > (self.__bounding_box.size_y/2):
            return 1 if y_offset > 0 else -1
        return 0

    def __bounding_box_sub_cb(self, data: BoundingBox2D):
        self.__bounding_box = BoundingBox2D
        self.__bounding_box_sub.unregister()
