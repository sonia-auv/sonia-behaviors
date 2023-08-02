from time import sleep
import rospy
from flexbe_core import EventState, Logger
from sonia_common.msg import FilterchainTarget

class detect_gate(EventState):

    def __init__(self):
        super().__init__(outcomes=['left_found', 'right_found', 'both_found', 'none_found', 'failed'],
                        input_keys=['calc_block'],
                        output_keys=['calc_block'])
        self.__left_seen = False
        self.__right_seen = False
        self.__left_counter = 0
        self.__right_counter = 0
        
    def on_enter(self, userdata):
        if len(userdata.calc_block) == 0:
            userdata.calc_block.append(0)
        def update_left(data):
            self.__left_counter += 1
            if self.__left_counter > 3:
                self.__left_seen = True
        def update_right(data):
            self.__right_seen = True
            self.__right_counter += 1
            if self.__right_counter > 3:
                self.__right_seen = True
        # This will be changed into the topic the white square
        self.__sub_left = rospy.Subscriber("/proc_image_processing/gate_left_target", FilterchainTarget, update_left)
        self.__sub_right = rospy.Subscriber("/proc_image_processing/gate_right_target", FilterchainTarget, update_right)

    
    def execute(self, userdata):
        sleep(2)
        if self.__left_seen and self.__right_seen:
            rtn_msg = 'both_found'
        elif self.__left_seen:
            rtn_msg = 'left_found'
        elif self.__right_seen:
            rtn_msg = 'right_found'
        else:
            rtn_msg = 'none_found'
        userdata.calc_block[0] += 1
        if userdata.calc_block[0] > 6:
            return 'failed'
        return rtn_msg

    def on_exit(self, userdata):
        self.__sub_left.unregister()
        self.__sub_right.unregister()
    