import subprocess
import threading
from networktables import NetworkTables

cond = threading.Condition()
notified = [False]


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


NetworkTables.initialize(server='10.xx.xx.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

# Insert your processing code here
print("Connected!")


### Java stdout

p = subprocess.Popen(["java", "MyClass2"], stdout=subprocess.PIPE)
line = p.stdout.readline()
while(line != "x\n"):
	print(line)
	line = p.stdout.readline()
