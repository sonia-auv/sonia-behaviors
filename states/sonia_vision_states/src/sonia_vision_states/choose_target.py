import rospy
from flexbe_core import EventState, Logger
from time import sleep
from sonia_common.msg import DetectionArray

class choose_target(EventState):
    """
    Get AI position on screen
        -- class_name               str     The name of the class of the image we are looking for
        -- nb_img                   int     Number of images needed calculate average. Defualts to 10.
        
        #> ai_pos                   list    Position on screen (x, y) of the object

        <= success                          If able to collect ai position
        <= failed                           If failed to collect ai position
    """
    def __init__(self, nb_img=10):
        super(choose_target, self).__init__(outcomes = ['earth', 'abydos', 'fail'],
                                     input_keys = ['gate_side'])

    def on_enter(self, userdata):
        Logger.log("Enter get gate side", Logger.REPORT_HINT)

    def on_exit(self, userdata):
        self.__obj_sub.unregister()

    def execute(self, userdata):
        if(userdata.gate_side == "Earth") :
            return 'earth'
        
        if(userdata.gate_side == "Abydos") :
            return 'abydos'
        
        return 'fail'