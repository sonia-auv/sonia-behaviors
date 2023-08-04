from flexbe_core import EventState, Logger

import rospy

from sonia_common.msg import MultiAddPose, AddPose

class pipe_align_translation(EventState):

    def __init__(self, tolerance=1, max_attempts=10):
        super().__init__(outcomes=["centered", "moved", "failed"],
                         input_keys=["function_constants_xy", "average_ctr", "counter"],
                         output_keys=["counter"])
        self.__tolerance = tolerance
        self.__max_attempts = max_attempts
        self.__axy = None
        self.__bxy = None
        self.__cxy = None
        self.__move_pub = None

    def on_enter(self, userdata):
        if len(userdata.counter) == 0:
            userdata.counter.append(0)
        self.__axy = userdata.function_constants_xy['a']
        self.__bxy = userdata.function_constants_xy['b']
        self.__cxy = userdata.function_constants_xy['c']
        self.__move_pub = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)

    def execute(self, userdata):
        if userdata.counter[0] > self.__max_attempts:
            return 'failed'
        
        x = userdata.average_ctr[0]
        y = userdata.average_ctr[1]
        
        if self.__move_sub(x, y):
            userdata.counter[0] += 1
            return "moved"

        return 'centered'

    def on_exit(self, userdata):
        pass
    
    def __move_sub(self, x, y, x_offset=300, y_offset=200) -> bool:
        pose = AddPose()
        pose.frame = 1
        
        if abs(x - x_offset) <= self.__tolerance and abs(y - y_offset) <= self.__tolerance:
            return False
        
        if abs(x - x_offset) > self.__tolerance:
            pose.position.x = self.__xy_function(x - x_offset)
            Logger.loghint(f"move in x: {x - x_offset}px, {pose.position.x}m")

        
        if abs(y - y_offset) > self.__tolerance:
            pose.position.y = self.__xy_function(y - y_offset)
            Logger.loghint(f"move in x: {y - y_offset}px, {pose.position.y}m")

        poses = MultiAddPose()
        poses.pose.append(pose)
        self.__move_pub.publish(poses)
        return True
            
    def __xy_function(self, delta_pixels):
        return round((self.__axy * delta_pixels * delta_pixels) + (self.__bxy * delta_pixels) + self.__cxy, 3)

   