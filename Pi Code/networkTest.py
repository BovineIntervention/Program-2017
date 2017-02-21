import sys
import time
from networktables import NetworkTables
import logging


logging.basicConfig(level = logging.DEBUG)

NetworkTables.setIPAddress("10.6.86.2") #172.22.11.2
NetworkTables.setClientMode()
NetworkTables.initialize()

sd = NetworkTables.getTable("SmartDashboard")

distance = 70
angle = 30
sd.putNumber("distance", distance)
sd.putNumber("angle", angle)


print sd.isConnected()
sd.shutdown()
