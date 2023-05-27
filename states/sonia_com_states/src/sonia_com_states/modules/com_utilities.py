# Com function libraries.
# Usefull for com states

#Includes
from sonia_common.msg import ModemUpdateMissionList

#Functions
def update_mission_array(mission_id, state):
    msg = ModemUpdateMissionList()

    msg.mission_id = mission_id
    msg.mission_state = state

    return msg