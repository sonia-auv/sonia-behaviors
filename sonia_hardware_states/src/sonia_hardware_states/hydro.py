import rospy
import math

# from Queue import deque
# from proc_mapping.srv import PingerLocationService
# from proc_control.srv import SetDecoupledTarget
# from proc_control.msg import TargetReached
# from std_msgs.msg import UInt8
# from sonia_common.msg import MpcInfo, MissionTimer

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils
from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose, PingAngles

class hydro(EventState):

    def __init__(self, frequency = 25000, speed=0, precision=0, long_rotation=False, timeout = 10):
        super(hydro, self).__init__(outcomes=['continue', 'failed'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])
        self.speed = speed
        self.precision = precision
        self.long_rotation = long_rotation
        self.pose = navUtils.addpose(0, 0, 0, 0, 0, 0, 1, speed, precision, long_rotation)
        self.hydro_angle = -1
        self.hydro_freq = -1
        self.timeout = timeout
        self.frequency = int(frequency)

    def get_hydro_data_cb(self, data):
        self.hydro_angle = data.heading
        self.hydro_freq = data.frequency
        self.is_alive = True

    def on_enter(self, userdata):
        self.is_alive = False
        self.get_hydro_data_sub = rospy.Subscriber('/proc_hydrophone/ping', PingAngles, self.get_hydro_data_cb)
        
        try:
            rospy.wait_for_message('/proc_hydrophone/ping', PingAngles, timeout=self.timeout)
            Logger.log('Frequency detected by hydro : ' + str(self.hydro_freq), Logger.REPORT_HINT)
            Logger.log('Angle detected by hydro : ' + str(self.hydro_angle), Logger.REPORT_HINT)
        except:
            Logger.log('No angles given by hydro', Logger.REPORT_HINT)
            pass
        
        self.pose = navUtils.addpose(0, 0, 0, 0, 0, self.hydro_angle, 1, self.speed, self.precision, self.long_rotation)
        pass

    def execute(self, userdata):
        if not self.is_alive:
            return 'failed'
        else :
            traj = userdata.input_traj
            new_traj = MultiAddPose()
            if not traj.pose:
                Logger.log('First position of the trajectory', Logger.REPORT_HINT)
            else:
                Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
                new_traj.pose = list(traj.pose)

            Logger.log(' yaw = ' + str(self.pose.orientation.z), Logger.REPORT_HINT)

            new_traj.pose.append(self.pose)
            userdata.trajectory = new_traj
            return 'continue'
      
    def on_exit(self, userdata):
        self.get_hydro_data_sub.unregister()
        pass