import subprocess
import threading
from networktables import NetworkTables
#!/usr/bin/env python3
#
# This is a NetworkTables client (eg, the DriverStation/coprocessor side).
# You need to tell it the IP address of the NetworkTables server (the
# robot or simulator).
#
# When running, this will create an automatically updated value, and print
# out the value.
#

import sys
import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging


class Listener():

    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)

        NetworkTables.initialize(server='10.36.23.2')

        self.sd = NetworkTables.getTable("SmartDashboard")
        self.auto_value = self.sd.getAutoUpdateValue("robotTime", 0)

        self.p = subprocess.Popen(["java", "MyClass2"], stdout=subprocess.PIPE)

    def fetchNetworkTable(self):
        print("robotTime:", self.auto_value.value)
        return self.auto_value.value

    def fetchStdout(self):
        line = self.p.stdout.readline()
        print(line)
        return line

    def updateState(self, robot):
        if (NetworkTables.isConnected()):
            robot.setPosition(self.fetchNetworkTable())
        else:
            robot.setPosition(self.fetchStdout())


if __name__ == "__main__":
    ey = Listener()
    ey.fetchStdout()
