from flexbe_core import EventState, Logger

import rospy

from sonia_common.msg import MultiAddPose, AddPose, FilterchainTargetAngle

class pipe_align_zoom(EventState):

    def __init__(self, target_size, tolerance=1, max_attempts=10):
        super().__init__(outcomes=["size ok", "moved", "failed"],
                         input_keys=["function_constants_z", "avg_size", "counter"],
                         output_keys=["counter"])
        self.__tolerance = tolerance
        self.__max_attempts = max_attempts
        self.__target_size = target_size
        self.__az = None
        self.__bz = None
        self.__cz = None
        self.__move_pub = None

    def on_enter(self, userdata):
        if len(userdata.counter) == 0:
            userdata.counter.append(0)
        self.__az = userdata.function_constants_z['a']
        self.__bz = userdata.function_constants_z['b']
        self.__cz = userdata.function_constants_z['c']
        self.__move_pub = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)

    def execute(self, userdata):

        if userdata.counter[0] > self.__max_attempts:
            return 'failed'
        
        avg_size = userdata.avg_size

        
        if self.__move_sub(avg_size - self.__target_size):
            userdata.counter[0] += 1
            return "moved"

        return 'size ok'

    def on_exit(self, userdata):
        self.__move_pub.unregister()

    def __move_sub(self, z) -> bool:
        pose = AddPose()
        pose.frame = 1

        if z <= self.__tolerance:
            return False
        pose.orientation.z = self.__z_function(z)

        poses = MultiAddPose()
        poses.pose.append(pose)
        self.__move_pub.publish(poses)
        return True

    def __z_function(self, delta_pixels):
        return round((self.__az * delta_pixels * delta_pixels) + (self.__bz * delta_pixels) + self.__cz, 3)