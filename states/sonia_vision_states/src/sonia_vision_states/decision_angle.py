
from flexbe_core import EventState, Logger


class decision_angle(EventState):

    def __init__(self):
        super(decision_angle, self).__init__(outcomes=['success'],
                         input_keys = ['calc_block'])

    def execute(self, userdata):
        if userdata.calc_block[0] > userdata.calc_block[1] :
            userdata.calc_block.append(userdata.calc_block[1])
        else :
            userdata.calc_block.append(-userdata.calc_block[1])
        
        return 'success'