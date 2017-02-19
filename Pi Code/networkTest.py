import sys
import time
from networktables import NetworkTables
import logging


logging.basicConfig(level = logging.DEBUG)

NetworkTables.setIPAddress("roborio-686-frc.local") #172.22.11.2
NetworkTables.setClientMode()
#NetworkTables.initialize()

NetworkTables.initialize(server = 'roborio-686-frc.local')



sd = NetworkTables.getTable("SmartDashboard")

distance = 70
angle = 30
sd.putNumber("distance", distance)
sd.putNumber("angle", angle)


print sd.isConnected()
sd.shutdown()
