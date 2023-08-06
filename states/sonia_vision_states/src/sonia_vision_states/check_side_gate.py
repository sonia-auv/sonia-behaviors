import rospy
from flexbe_core import EventState, Logger
from time import sleep
from sonia_common.msg import DetectionArray

class check_gate_side(EventState):
    """
    Get AI position on screen
        -- class_name               str     The name of the class of the image we are looking for
        -- nb_img                   int     Number of images needed calculate average. Defualts to 10.
        
        #> ai_pos                   list    Position on screen (x, y) of the object

        <= success                          If able to collect ai position
        <= failed                           If failed to collect ai position
    """
    def __init__(self, nb_img=10):
        super(check_gate_side, self).__init__(outcomes = ['success', 'failed'],
                                     output_keys = ['gate_side'])
        self.__nb_img = nb_img
        self.__queue_earth = []
        self.__queue_abydos = []
        self.__nb_attempts = 0

    def on_enter(self, userdata):
        Logger.log("Enter get gate side", Logger.REPORT_HINT)
        self.__obj_sub = rospy.Subscriber("/proc_detection/bounding_box", DetectionArray, self.__filterchain_obj_sub_cb)

    def on_exit(self, userdata):
        self.__obj_sub.unregister()

    def execute(self, userdata):
        
        while True:
            if self.__nb_attempts > 10:
                Logger.logerr("Too many attempts to get images")
                return 'failed'
            sleep(1)
            if len(self.__queue) >= self.__nb_img:
                break
            self.__nb_attempts += 1
        
        somme_e = 0
        somme_a = 0

        for i in self.__queue_abydos :
            somme_a += (i.left + i.right) / 2
        somme_a = somme_a/len(self.__queue_abydos)

        for i in self.__queue_earth :
            somme_e += (i.left + i.right) / 2
        somme_e = somme_e/len(self.__queue_earth)

        if(abs(0.5-somme_e)<abs(0.5-somme_a)) :
            userdata.gate_side = "Earth"
        else :
            userdata.gate_side = "Abydos"
      
        
        Logger.loghint(f"AI target aquired {userdata.ai_pos!r}")
            
        return 'success'


    def __filterchain_obj_sub_cb(self, msg: DetectionArray):
        if len(self.__queue) < self.__nb_img:
            for i in msg.detected_object :
                if i.class_name == "Gate_earth" :
                    self.__queue_earth.append(i)
                
                if i.class_name == "Gate_abydos" :
                    self.__queue_abydos.append(i)
