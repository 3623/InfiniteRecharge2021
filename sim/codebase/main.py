# Drive the robot according to user key inputs
# The user can drag and drop paths
# Have the robot be able to follow a few paths(even if it’s not your final algorithm)


from cmu_112_graphics import *
from tkinter import *
from robot import RobotModel
import Utils
import math
from controls import Controls
import threading
import time
from logger import Logger
from graph import StackedTimeGraph


class SimulationApp(App):

    def appStarted(self):
        # Citation: this is a modified version of image from user BENL at link
        # https://www.chiefdelphi.com/t/top-down-orthographic-view-of-frcs-2019-playing-field/335397/9
        self._fieldImage = self.loadImage("2019-field-blue.png")
        self.fieldImageScaled = ImageTk.PhotoImage(self._fieldImage)
        # Citation: from cleanpng.com, contributor: georgen
        url = "https://icon2.cleanpng.com/20180417/bxw/kisspng-steve-harvey-still-trippin-act-like-a-lady-thin-steve-jobs-5ad626cde40c49.3709911715239840779341.jpg"
        self._robotImage = self.scaleImage(self.loadImage(url), 0.6)
        self._WAYPOINT_RADIUS = 30
        self.setAppDims()
        self.resetUserInputs()

        self.FIELD_REAL_WIDTH = 8.23  # meters
        self.FIELD_REAL_HEIGHT = 9  # 16.46

        self.timerDelay = 30  # milliseconds

        self.robot = RobotModel(1.0, 1.0, 0.0)
        self.leftVoltage, self.rightVoltage = 0.0, 0.0
        self.simTime = 0.0
        self.ODOMETRY_UPDATE_RATE = 100
        odometryThread = threading.Thread(
            target=self.odometryPeriodic, daemon=True)
        odometryThread.start()

        self.waypoints = []

        self.controls = Controls(self.waypoints, self.robot)
        self.CONTROLS_UPDATE_RATE = 100
        controlThread = threading.Thread(
            target=self.controlsPeriodic, daemon=True)
        controlThread.start()

        self.logger = Logger()
        self.logger.registerLoggerDict(self.robot.logDict, "robot")
        self.logger.registerLoggerDict(self.controls.logDict, "controls")
        yAxes = [self.logger.dict["robot.heading"],
                 self.logger.dict["robot.vel"],
                 self.logger.dict["controls.ffSpeed"]]
        self.graph = StackedTimeGraph(self.logger.time, yAxes,
                           (self.fieldImageWidth, self.width), (self.height, 0))

    def resetUserInputs(self):
        self._appTime = 0
        self.releaseDelay = 0.1
        self.inputKeys = {"Up": KeyLatch(self.releaseDelay),
                          "Down": KeyLatch(self.releaseDelay),
                          "Right": KeyLatch(self.releaseDelay),
                          "Left": KeyLatch(self.releaseDelay)}
        self.autoDriving = False
        self.autoDrivingStart = False
        self.selectedWaypoint = None
        self.rotatingWaypoint = False
        self.lastClickTime = 0
        self._DOUBLE_CLICK_TIME = 0.2

    def timerFired(self):
        deltaTime = self.timerDelay/1000.0
        self._appTime += deltaTime
        if self.autoDriving:
            self.logger.log(self.simTime)

    def odometryPeriodic(self):
        while self._running:
            deltaTime = 1.0/self.ODOMETRY_UPDATE_RATE
            self.simTime += deltaTime
            self.robot.updateVoltage(
                self.leftVoltage, self.rightVoltage, deltaTime)
            self.robot.updatePositionWithVelocity(deltaTime)
            time.sleep(deltaTime)

    def controlsPeriodic(self):
        while self._running:
            if self.autoDriving and self.waypoints:
                self.driveUsingPursuit()
            else:
                self.driveUsingKeys()
            time.sleep(1.0/self.CONTROLS_UPDATE_RATE)

    def driveUsingPursuit(self):
        if self.autoDrivingStart:
            self.autoDrivingStart = False
            self.simTime = 0.0
            self.logger.clear()
            startPoint = self.waypoints[self.controls.index]
            self.robot.center.setPosition(
                startPoint.x, startPoint.y, heading=startPoint.heading)
        self.leftVoltage, self.rightVoltage = self.controls.updatePursuit()

    def driveUsingKeys(self):
        avgVoltage = (self.leftVoltage + self.rightVoltage) / 2.0
        turnVoltage = 2.0
        if self.inputKeys["Up"].getIsActive(self._appTime):
            avgVoltage += 0.9
        elif self.inputKeys["Down"].getIsActive(self._appTime):
            avgVoltage -= 0.9
        else:
            turnVoltage = 4.0
            if avgVoltage > 0.0:
                avgVoltage -= 2.0
                if avgVoltage < 0.0:
                    avgVoltage = 0.0
            elif avgVoltage < 0.0:
                avgVoltage += 2.0
                if avgVoltage > 0.0:
                    avgVotlage = 0.0

        self.leftVoltage = Utils.limit(avgVoltage, 10.0, -10.0)
        self.rightVoltage = Utils.limit(avgVoltage, 10.0, -10.0)

        if self.inputKeys["Right"].getIsActive(self._appTime):
            self.leftVoltage += turnVoltage
            self.rightVoltage -= turnVoltage
        elif self.inputKeys["Left"].getIsActive(self._appTime):
            self.leftVoltage -= turnVoltage
            self.rightVoltage += turnVoltage

    def redrawAll(self, canvas):
        canvas.create_image(self.fieldImageWidth/2, self.height/2,
                            image=self.fieldImageScaled)
        robotAppX, robotAppY = self.realWorldToAppCoords(
            self.robot.center.x, self.robot.center.y)
        rotatedRobot = self._robotImage.rotate(-self.robot.center.heading)
        canvas.create_image(robotAppX, robotAppY,
                            image=ImageTk.PhotoImage(rotatedRobot))

        for i, waypoint in enumerate(self.waypoints):
            self.drawNode(canvas, waypoint, i)

        paths = self.controls.getPath()

        for x, path in enumerate(paths):
            if x == 0:
                color = "yellow"
            else:
                color = "white"
            for i in range(len(path)-1):  # len(path)-1):
                point = path[i]
                pX1, pY1 = self.realWorldToAppCoords(point[0], point[1])
                nextPoint = path[i+1]
                pX2, pY2 = self.realWorldToAppCoords(
                    nextPoint[0], nextPoint[1])
                canvas.create_line(pX1, pY1, pX2, pY2, fill=color)

        self.graph.draw(canvas)

    def keyPressed(self, event):
        key = event.key
        if key == "h":
            self.superhelp()
        elif key in self.inputKeys:
            self.inputKeys[key].lastEventRelease = False
        elif key == "Delete":
            if self.selectedWaypoint is not None:
                self.waypoints.remove(self.selectedWaypoint)
                self.selectedWaypoint = None
        elif key == "r":
            self.autoDriving = False
        elif key == "a":
            if not self.autoDriving:
                self.autoDrivingStart = True
            self.autoDriving = not self.autoDriving
            self.controls.reset()
        elif key == "w":
            self.incrementWaypointSpeed(0.05)
        elif key == "s":
            self.incrementWaypointSpeed(-0.05)
        elif key == "p":
            print(self.logger.dict["robot.vel"])
        else:
            None

    def keyReleased(self, event):
        key = event.key
        if key in self.inputKeys:
            self.inputKeys[key].lastEventRelease = True
            self.inputKeys[key].timeLastRelease = self._appTime
        else:
            None

    def mousePressed(self, event):
        if event.x < self.fieldImageWidth:
            self.fieldClick(event)

    def fieldClick(self, event):
        self.cursorX, self.cursorY = event.x, event.y
        self.selectedWaypoint = None
        for waypoint in self.waypoints:
            appWaypointX, appWaypointY = self.realWorldToAppCoords(
                waypoint.x, waypoint.y)
            if Utils.distance(appWaypointX, appWaypointY, event.x, event.y) < self._WAYPOINT_RADIUS:
                self.selectedWaypoint = waypoint
                newAngle = math.degrees(-math.atan2(appWaypointX - event.x,
                                                    appWaypointY - event.y))
                if self._appTime - self.lastClickTime < self._DOUBLE_CLICK_TIME:
                    waypoint.isCritical = not waypoint.isCritical
                if abs(newAngle - self.selectedWaypoint.heading) < 40.0:
                    self.rotatingWaypoint = True
                else:
                    self.rotatingWaypoint = False

        if self.selectedWaypoint is None:  # New waypoint
            x, y = self.appToRealWorldCoords(event.x, event.y)
            newWaypoint = Waypoint(x, y, 0.0, 0.6)
            self.waypoints.append(newWaypoint)
            self.selectedWaypoint = newWaypoint

        self.lastClickTime = self._appTime

    def fieldDragged(self, event):
        dX = event.x - self.cursorX
        dY = event.y - self.cursorY
        if self.selectedWaypoint is not None:
            appWaypointX, appWaypointY = self.realWorldToAppCoords(
                self.selectedWaypoint.x, self.selectedWaypoint.y)
            if self.rotatingWaypoint:
                newAngle = math.degrees(-math.atan2(appWaypointX - event.x,
                                                    appWaypointY - event.y))
                self.selectedWaypoint.setPosition(heading=newAngle)
            else:
                appWaypointX += dX
                appWaypointY += dY
                waypointX, waypointY = self.appToRealWorldCoords(
                    appWaypointX, appWaypointY)
                self.selectedWaypoint.x, self.selectedWaypoint.y = waypointX, waypointY
        self.cursorX, self.cursorY = event.x, event.y

    def mouseDragged(self, event):
        if event.x < self.fieldImageWidth:
            self.fieldDragged(event)

    def mouseReleased(self, event):
        None


    def mouseMoved(self, event):
        self.graph.updateHover(event.x)

    def realWorldToAppCoords(self, x, y):
        newX = (self.fieldImageWidth/2) + (self.fieldImageWidth/self.FIELD_REAL_WIDTH*x)
        newY = (self.height) - (self.height/self.FIELD_REAL_HEIGHT*y)
        return int(newX), int(newY)

    def appToRealWorldCoords(self, x, y):
        newX = (x - self.fieldImageWidth/2) / (self.fieldImageWidth/self.FIELD_REAL_WIDTH)
        newY = (self.height - y) / (self.height/self.FIELD_REAL_HEIGHT)
        return newX, newY

    def setAppDims(self):
        root = self._root
        screenWidth = root.winfo_screenwidth()
        screenHeight = root.winfo_screenheight()
        imageWidth, imageHeight = self._fieldImage.size
        if screenHeight/imageHeight < screenWidth/imageWidth:
            scaleFactor = screenHeight/imageHeight*0.9
        else:
            scaleFactor = screenWidth/imageWidth*0.9

        self.height = int(imageHeight * scaleFactor)
        self.width = int(imageWidth * scaleFactor + screenWidth * 0.3)
        self.setSize(self.width, self.height)
        scaledFieldImage = self.scaleImage(self._fieldImage, scaleFactor)
        self.fieldImageWidth = scaledFieldImage.size[0]
        self.fieldImageScaled = ImageTk.PhotoImage(scaledFieldImage)

    def drawNode(self, canvas, node, i):
        r = self._WAYPOINT_RADIUS
        x, y = self.realWorldToAppCoords(node.x, node.y)
        color = self.numberToColor(node.kSpeed)
        r2 = r
        if node.isCritical:
            canvas.create_oval(x+r, y+r,
                               x-r, y-r,
                               fill="white")
            r2 = 0.7*r
        canvas.create_oval(x+r2, y+r2,
                           x-r2, y-r2,
                           fill=color,
                           outline="white")
        x1 = x + (r * 1.3 * math.sin(node.r))
        x2 = x + (r * 0.3 * math.sin(node.r))
        y1 = y - (r * 1.3 * math.cos(node.r))
        y2 = y - (r * 0.3 * math.cos(node.r))
        canvas.create_line(x2, y2, x1, y1, width=r/4, fill="gold")
        canvas.create_text(x, y, anchor="c", text=f"{i}")

    def numberToColor(self, x):
        scaled = 255 - abs(int(x * 255))
        red, green, blue = scaled, scaled, scaled
        if x < 0.0:
            red = 255
        elif x > 0.0:
            green = 255
        # set your favourite rgb color
        color = '#%02x%02x%02x' % (red, green, blue)
        return color

    def incrementWaypointSpeed(self, delta):
        if self.selectedWaypoint is not None:
            speed = self.selectedWaypoint.kSpeed
            speed += delta
            speed = Utils.limit(speed, 1.0, -1.0)
            self.selectedWaypoint.kSpeed = speed

    def superhelp(self):
        print("Arrow keys to move.\n"
        +     "Click on the screen to create a waypoint, and click\n"
        +     "  on any already created waypoint to select it.\n"
        +     "  Double clicking on the waypoint to make it critical \n"
        +     "  (the robot will slow down and stop there, else \n"
        +     "  it will just drive through it). The yellow tick \n"
        +     "  indicates the direction of the waypoint, and can be\n"
        +     "  dragged to change the direction. The speed is\n"
        +     "  indicated by the color and controlled by 'w' and 's'.\n"
        +     "  The waypoint can be deleted by 'del'.\n"
        +     "To start the robot autodriving, there must be 2 waypoints\n"
        +     "  and a path should be shown in white. Then 'a' will start\n"
        +     "  and stop autodriving. 'r' will reset the robot.")


class Waypoint(Utils.Twist):

    def __init__(self, x, y, heading, kSpeed, isCritical=False):
        super().__init__(x, y, heading)
        self.kSpeed = kSpeed
        self.isCritical = isCritical

    def toString(self):
        return f"x: {round(self.x,2)}, y: {round(self.y,2)}, " + \
               f"heading: {round(self.heading,2)}, speed: {self.kSpeed}," + \
               f"critical: {self.isCritical}"

    def __repr__(self):
        return self.toString()


class KeyLatch():

    def __init__(self, delay):
        self.timeLastRelease = -delay
        self.delay = delay
        self.lastEventRelease = True

    def getIsActive(self, time):
        if self.lastEventRelease == False:
            return True
        else:
            if time - self.timeLastRelease < self.delay:
                return True
            else:
                return False


if __name__ == "__main__":
    SimulationApp()
