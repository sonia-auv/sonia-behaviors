# Navigation function libraries.
# Usefull for navigation states

# includes
from sonia_common.msg import AddPose, MissionTimer

# fill and return an addpose object 
def addpose( x, y, z, rx, ry,rz, frame, speed, fine, rot):
        buffer = AddPose()
        buffer.position.x = x
        buffer.position.y = y
        buffer.position.z = z
        buffer.orientation.x = rx
        buffer.orientation.y = ry
        buffer.orientation.z = rz
        buffer.frame = frame
        buffer.speed = speed
        buffer.fine = fine
        buffer.rotation = rot

        return buffer

def missionTimerFunc(mission, timeout, uniqueID, status):
        buffer = MissionTimer()
        buffer.mission = mission
        buffer.timeout = timeout
        buffer.uniqueID = uniqueID
        buffer.status = status
        return buffer