import math
import Utils

# Class has methods which control the robots movement along
# a path specified by a list of waypoints


class Controls():

    def __init__(self, waypoints, robot):
        self.robot = robot
        self.waypoints = waypoints
        self.index = 0
        self.isFinished = False

        self.UPDATE_RATE = 100

        self.kRadiusCritical = 0.1
        self.kEpsilonCritical = 3.0
        self.kRadiusPath = 0.0
        self.kEpsilonPath = 5.0

        self.MAX_SPEED = 3.3

        self.kV = 1.0 / 14.0 * 14.0
        self.kTurn = 12.0 / 450.0

        self.logDict = {"index": None,
                        "waypointX": None,
                        "waypointY": None,
                        "ffSpeed": None}

    def reset(self):
        self.index = 0
        self.isFinished = False

    def updatePursuit(self):
        if self.index > len(self.waypoints) - 1:
            self.index = 0
        curWaypoint = self.waypoints[self.index]
        robotPose = self.robot.center

        feedForwardSpeed = curWaypoint.kSpeed
        distanceToWaypoint = Utils.distance(robotPose.x, robotPose.y,
                                            curWaypoint.x, curWaypoint.y)
        debug = False
        # important to be at exactly
        if curWaypoint.isCritical or self.index == len(self.waypoints) - 1:

            if distanceToWaypoint < abs(feedForwardSpeed) * 1.2:
                # speed reduces as distance gets smaller
                feedForwardSpeed = math.copysign(
                    distanceToWaypoint / 1.2, feedForwardSpeed)
                if abs(feedForwardSpeed) < 0.25:
                    feedForwardSpeed = math.copysign(0.25, feedForwardSpeed)

            if Utils.withinThreshold(distanceToWaypoint, 0.0, self.kRadiusCritical) or self.isFinished:
                debug = True
                feedForwardSpeed = 0.0
                # at point and heading, we're done
                if Utils.withinThreshold(robotPose.heading, curWaypoint.heading, self.kEpsilonCritical):
                    if not self.isFinished:
                        print(
                            f"At Waypoint: {self.index} ({curWaypoint.toString()})")
                    if self.index == len(self.waypoints) - 1 or self.isFinished:
                        if not self.isFinished:
                            print("Finished Path Following")
                        self.isFinished = True
                        return 0.0, 0.0
                    else:
                        self.index += 1
                        curWaypoint = self.waypoints[self.index]

                else:
                    # at point but not heading, just turn to the point
                    # ptrOutput = DrivetrainControls.turnToAngle(
                    #     curWaypoint.heading, robotPose.heading)
                    # return curvatureDrive(0.0, ptrOutput, True)
                    return 0.0, 0.0

        elif Utils.withinThreshold(distanceToWaypoint, 0.0, self.kRadiusPath) \
                and Utils.withinThreshold(robotPose.heading, curWaypoint.heading, self.kEpsilonPath):
            # at non-critical waypoint
            print(f"At Waypoint: {self.index} ({curWaypoint.toString()})")
            self.index += 1
            curWaypoint = self.waypoints[self.index]
            debug = True

        # if not in a special case, just run path following
        self.logDict["ffSpeed"] = feedForwardSpeed
        return self.pathFollowing(feedForwardSpeed)

    def pathFollowing(self, ffSpeed):
        curWaypoint = self.waypoints[self.index]
        robotPose = self.robot.center
        a, b = self.getPathGeometry(robotPose, curWaypoint)

        nextSpeed = ((self.MAX_SPEED * ffSpeed) * 0.1) + \
            (robotPose.velocity * 0.9)
        deltaX = nextSpeed / self.UPDATE_RATE
        if (deltaX < 0.0) == (ffSpeed > 0.0):
            deltaX = 0.0

        if deltaX != 0.0:
            y2 = self.getYFromCoeffs(a, b, deltaX)
            hypot = Utils.hypotenuseLength(deltaX, y2)
            ratio = abs(deltaX / hypot)
            deltaX *= ratio

        self.kRadiusPath = abs(deltaX) * self.UPDATE_RATE * 0.1
        dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX)
        relativeFeedForwardAngle = math.atan(dx2)

        if False:
            # print(f"{relativeAdjacDist} {relativeOpposDist} {relativeGoalDeriv}")
            print(f"{a} {b} {deltaX}")

        turnOutput = -math.degrees(relativeFeedForwardAngle) * \
            self.UPDATE_RATE * self.kTurn
        turnLimitedFFSpeed = math.copysign(
            abs(ffSpeed)-abs(turnOutput/12.0), ffSpeed)
        self.logDict["ffSpeed"] = turnLimitedFFSpeed
        outputLeft = (turnLimitedFFSpeed * self.kV * 12.0) + turnOutput
        outputRight = (turnLimitedFFSpeed * self.kV * 12.0) - turnOutput

        return outputLeft, outputRight

    def getPathGeometry(self, robotPose, curWaypoint, returnPath=False):
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
                x += self.MAX_SPEED*curWaypoint.kSpeed/self.UPDATE_RATE
            return path

    def generateSpline(self, x, y, dx):
        a = ((x * dx) - (2 * y)) / (x * x * x)
        b = ((3 * y) - (dx * x)) / (x * x)
        return a, b

    def getYFromCoeffs(self, a, b, x):
        return (a * x**3) + (b * x**2)

    def getPath(self):
        path = []
        if len(self.waypoints) > 1 and self.index != 0:
            path.append(self.getPathGeometry(
                self.robot.center, self.waypoints[self.index], True))
        else:
            path.append([])
        for i, waypoint in enumerate(self.waypoints[:-1]):
            path.append(self.getPathGeometry(
                waypoint, self.waypoints[i+1], True))

        return path
