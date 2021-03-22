import Utils
import math

# Models a differential drive robot movement and motors


class RobotModel(Utils.Twist):

    def __init__(self, *args):
        super().__init__(*args)
        self.logDict = {"x": None,
                        "y": None,
                        "heading": None,
                        "vel": None,
                        "angVel": None,
                        "leftVel": None,
                        "rightVel": None}

    def zero(self):
        self.leftSide.velocity = 0.0
        self.leftSide.acceleration = 0.0
        self.rightSide.velocity = 0.0
        self.rightSide.acceleration = 0.0

    def getPathGeometry(self, robotPose, curWaypoint, returnPath=False):
        kLINE_LENGTH = 0.1
        distanceToWaypoint = Utils.distance(robotPose.x, robotPose.y,
                                            curWaypoint.x, curWaypoint.y)

        straightPathAngle = math.atan2(
            curWaypoint.x - robotPose.x, curWaypoint.y - robotPose.y)
        relativeAngle = robotPose.r - straightPathAngle
        relativeOpposDist = distanceToWaypoint * math.sin(relativeAngle)
        relativeAdjacDist = distanceToWaypoint * math.cos(relativeAngle)
        relativeGoalAngle = robotPose.r - curWaypoint.r
        relativeGoalAngle = Utils.limit(relativeGoalAngle,
                                        math.pi*0.3, -math.pi*0.3)
        relativeGoalDeriv = math.tan(relativeGoalAngle)
        a, b = self.generateSpline(
            relativeAdjacDist, relativeOpposDist, relativeGoalDeriv)
        if not returnPath:
            return a, b
        else:
            path = []
            cos = math.cos(robotPose.r)
            sin = math.sin(robotPose.r)
            x = 0
            while abs(x) <= abs(relativeAdjacDist) \
                    and not Utils.withinThreshold(curWaypoint.kSpeed, 0.0, 0.01):
                y = self.getYFromCoeffs(a, b, x)
                globalX = (x * sin) - (y * cos) + robotPose.x
                globalY = (y * sin) + (x * cos) + robotPose.y
                path.append((globalX, globalY))
                x += kLINE_LENGTH
            return path

    def generateSpline(self, x, y, dx):
        a = ((x * dx) - (2 * y)) / (x * x * x)
        b = ((3 * y) - (dx * x)) / (x * x)
        return a, b

    def getYFromCoeffs(self, a, b, x):
        return (a * x**3) + (b * x**2)

    def getPath(self, waypoints, index):
        path = []
        if len(waypoints) > 1 and index != 0:
            path.append(self.getPathGeometry(
                self.robot, waypoints[index], True))
        else:
            path.append([])
        for i, waypoint in enumerate(waypoints[:-1]):
            path.append(self.getPathGeometry(
                waypoint, waypoints[i+1], True))

        return path

    # def updatePositionWithVelocity(self, deltaTime):
    #     self.logDict["x"] = self.center.x
    #     self.logDict["y"] = self.center.y
    #     self.logDict["heading"] = self.center.heading
    #     self.logDict["vel"] = self.center.velocity
    #     self.logDict["angVel"] = self.center.angularVelocity
    #     self.logDict["leftVel"] = self.leftSide.velocity
    #     self.logDict["rightVel"] = self.rightSide.velocity

    #     # print(self.leftSide.velocity, self.rightSide.velocity)

