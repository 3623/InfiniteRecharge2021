/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controls;

import java.util.LinkedList;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DrivetrainModel;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Tuple;
import frc.util.Utils;

/**
 * Add your docs here.
 */
public class CubicSplineFollower {
    private final double WHEEL_BASE;
    private static final double UPDATE_RATE = 200.0;

    private LinkedList<Waypoint> waypoints;
    private Waypoint curWaypoint = null;

    public Boolean isFinished = false;

    private double kMaxAccel = 0.2; // m/s^2 * 200
    private final double kNextSpeedFactor = 0.02; // 0.0 - 1.0
    private final double kTurnMultiplier = 1.0;
    private final double kMaxAngularDiffFactor = 3.0; // m/s * 2
    private final double kSlowdownRadius = 1.0; // m
    private final double kMinApproachSpeedCritical = 0.2; // %
    private final double kRadiusCritical = 0.1;; // m
    private final double kScaleRadiusPath = 1; // constant
    private double kRadiusPath = 0.0; // this updates dynamically
    // deg, keeping this because this dictates when the robot switches
    private final double kAngularErrorPath = 20.0;
    private final double kMaxSplineAngle = Math.PI * 0.3;

    private DrivetrainModel drivetrainState;

    private double maxSpeed;
    private double ffSpeed = 0.0;
    private double maxTurn = 0.0;

    private static final Boolean debug = false;

    public CubicSplineFollower(DrivetrainModel drivetrain) {
        drivetrainState = drivetrain;
        WHEEL_BASE = DrivetrainModel.WHEEL_BASE;

        waypoints = new LinkedList<Waypoint>();
    }

    /**
     * Updates the path follower with a new robot pose. Should be called at rate
     * equal to {@code UPDATE_RATE}.
     *
     * @param robotPose the current robot pose, with position and velocities
     * @return a tuple with left and right wheel speeds, n/s
     */
    public Tuple updatePursuit(Pose robotPose) {
        boolean finished = waypoints.isEmpty() && curWaypoint == null;
        if (finished && !isFinished) System.out.println("Finished Path Following");
        isFinished = finished;
        if (isFinished) return new Tuple(0.0, 0.0);
        if (curWaypoint == null) curWaypoint = waypoints.pollFirst();
        double distanceFromWaypoint = Geometry.distance(robotPose, curWaypoint);
        if (curWaypoint.getGear() == true) drivetrainState.shiftMode(true);
        else drivetrainState.shiftMode(false);
        maxSpeed = drivetrainState.topSpeed;
        ffSpeed = curWaypoint.speed();
        boolean nextWaypoint = false;
        if (debug) System.out.println("Seeking: " + curWaypoint.toString());
        if (curWaypoint.isCritical) { // important to be at exactly
            if (distanceFromWaypoint < Math.abs(ffSpeed) * kSlowdownRadius) {
                // speed reduces as distance gets smaller
                if (debug) System.out.println("Slowing down");
                ffSpeed = Math.copySign(distanceFromWaypoint / kSlowdownRadius, ffSpeed);
                maxTurn *= (distanceFromWaypoint / kSlowdownRadius); // TODO so too
                if (Math.abs(ffSpeed) < kMinApproachSpeedCritical) // TODO this might not be necessary
                    ffSpeed = Math.copySign(kMinApproachSpeedCritical, ffSpeed);
            }
            if (distanceFromWaypoint < kRadiusCritical || isFinished) nextWaypoint = true;
                // at point and heading, we're done
        } else if (distanceFromWaypoint < kRadiusPath
                && Utils.withinThreshold(robotPose.heading, curWaypoint.heading, kAngularErrorPath))
                // at non-critical waypoint
                nextWaypoint = true;

        if (nextWaypoint) {
            System.out.println("At Waypoint: " + curWaypoint.toString());
            curWaypoint = waypoints.pollFirst();
            return updatePursuit(robotPose);
        }
        // if not in a special case, just run path following
        return pathFollowing(robotPose);
    }

    /**
     * Uses a cubic spline calculated OTF to figure out a projected change in angle
     * required to follow path and uses this as a feed forward value in conjuction
     * with a d term used to cancel out rotational inertia of the robot. This method
     * cheats by setting the initial point of the cubic spline as x=0, y=0, dx=0 to
     * make calculations simpler. This means that the waypoint has to be converted
     * to local coordinates in reference to the robot.
     *
     * @return a tuple of left and right output linear speed
     */
    public Tuple pathFollowing(Pose robotPose) {
        Tuple pathCoefficients = getPathGeometry(robotPose, curWaypoint);
        double a = pathCoefficients.left;
        double b = pathCoefficients.right;
        double nextSpeed = ((maxSpeed * ffSpeed) * kNextSpeedFactor) +
                            (robotPose.velocity * (1.0-kNextSpeedFactor));
        double deltaX = nextSpeed / UPDATE_RATE;
        if (Math.signum(deltaX) != Math.signum(ffSpeed))
            deltaX = 0.0;
        /*
         * Average of ffSpeed and actual speed scaled by cosine (to account for how far
         * off straight the robot has to drive) and cos again (the further off straight
         * the longer the curve) then divided by update rate (to get deltaX, the
         * position along the spline the robot will be at for the next update, giving a
         * feed forward point). If this just used actual speed, a stopped robot would
         * not look ahead.
         */

        if (deltaX != 0.0) {
            double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
            double hypot = Geometry.hypotenuse(deltaX, y2);
            double ratio = Math.abs(deltaX / hypot);
            deltaX *= ratio;
        }

        kRadiusPath = Math.abs(deltaX) * UPDATE_RATE * kScaleRadiusPath;
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relativeFFAngle = Math.atan(dx2);
        if (debug) System.out.println(relativeFFAngle);
        double omega = relativeFFAngle * UPDATE_RATE;

        // Convert from derivative to angle

        double desiredSpeed = ffSpeed * maxSpeed;
        if (debug) System.out.println(robotPose.velocity);
        if (debug) System.out.println(desiredSpeed + "raw");
        if (desiredSpeed - robotPose.velocity > kMaxAccel)
            desiredSpeed = robotPose.velocity + kMaxAccel;
        else if (desiredSpeed - robotPose.velocity < -kMaxAccel)
            desiredSpeed = robotPose.velocity - kMaxAccel;
        if (debug) System.out.println(desiredSpeed + "accel");
        double lrSpeedDifference = omega * WHEEL_BASE * kTurnMultiplier;
        maxTurn = kMaxAngularDiffFactor * Math.abs(ffSpeed);
        if (true) System.out.println("Max turn: " + maxTurn);
        lrSpeedDifference = Utils.limit(lrSpeedDifference, maxTurn, -maxTurn);
        if (desiredSpeed + Math.abs(lrSpeedDifference) > maxSpeed)
            desiredSpeed = maxSpeed - Math.abs(lrSpeedDifference);
        else if (desiredSpeed - Math.abs(lrSpeedDifference) < -maxSpeed)
            desiredSpeed = -maxSpeed + Math.abs(lrSpeedDifference);
        if (debug) System.out.println(desiredSpeed  + "turn");
        double leftSpeed = desiredSpeed - (lrSpeedDifference / 2);
        double rightSpeed = desiredSpeed + (lrSpeedDifference / 2);
        if (debug) System.out.println(desiredSpeed + " " + lrSpeedDifference);
        if (debug) System.out.println(leftSpeed + " " + rightSpeed);
        return new Tuple(leftSpeed, rightSpeed);
    }

    /**
     * Calculates the relative angles and distances from the current robot position
     * to the desired goal point.
     *
     * @param startPoint the start position of the robot, if using dynamic path
     *                   generation, this should be the robot position
     * @param goalPoint  the goal position of the path to be calculated
     * @return a tuple of path coefficients a and b respectively for a cubic spline
     */
    private Tuple getPathGeometry(Pose startPoint, Pose goalPoint) {
        double distanceFromWaypoint = Geometry.distance(startPoint, goalPoint);
        double straightPathAngle = Math.atan2(goalPoint.x - startPoint.x, goalPoint.y - startPoint.y);
        double relativeAngle = startPoint.r - straightPathAngle;

        double relativeOpposDist = distanceFromWaypoint * Math.sin(relativeAngle);
        double relativeAdjacDist = distanceFromWaypoint * Math.cos(relativeAngle);
        double relativeGoalAngle = startPoint.r - goalPoint.r;
        relativeGoalAngle = Geometry.limitAngleRad(relativeGoalAngle);
        relativeGoalAngle = Utils.limit(relativeGoalAngle, kMaxSplineAngle, -kMaxSplineAngle);
        double relativeGoalDeriv = Math.tan(relativeGoalAngle);
        if (debug) {
            System.out.println(relativeAdjacDist + " " + relativeOpposDist + " " + relativeGoalDeriv);
        }
        return generateSpline(relativeAdjacDist, relativeOpposDist, relativeGoalDeriv);
    }

    /**
     * Calculates the value of two coefficients (a & b) of a cubic spline specified
     * by two points and derivatives.
     *
     * @Note The first point is assumed to be (0, 0) with a derivative of 0. Second
     *       point must be in reference to this point
     * @param x  the x coordinate of the second point
     * @param y  the y coordinate of the second point
     * @param dx the desired slope of the second point
     * @implNote Not complicated, just two equations derived from solving the system
     *           of equations where x1=0, y1=0, and dx1=0, and x2, y2, and dx2 are
     *           specified in relation to p1, and y=ax^3+bx^2+cx+d (c and d are
     *           equal to 0 because of definition)
     * @return a tuple for coefficients a and b respectively
     */
    private static Tuple generateSpline(double x, double y, double dx) {
        double a = ((x * dx) - (2 * y)) / (x * x * x);
        double b = ((3 * y) - (dx * x)) / (x * x);
        return new Tuple(a, b);
    }

    /**
     * Clears the array list of waypoints and resets index so that the path follower
     * can be used again
     */
    public void clearWaypoints() {
        waypoints.clear();
    }

    /**
     * Returns the current waypoint being followed by the path follower
     *
     * @return {@link Waypoint}
     */
    public Waypoint getCurrentWaypoint() {
        return curWaypoint;
    }

    /**
     * Adds a waypoint to the list of waypoints (FILO)
     *
     * @param newWaypoint see {@link Waypoint}
     */
    public void addWaypoint(Waypoint newWaypoint) {
        waypoints.add(newWaypoint);
    }

    public void addWaypoint(double x, double y, double heading, double speed, Boolean isCritical) {
        waypoints.add(new Waypoint(x, y, heading, speed, isCritical));
    }

    public void addWaypoint(Pose pose, double speed, Boolean isCritical) {
        waypoints.add(new Waypoint(pose, speed, isCritical));
    }

    public void addWaypoint(double x, double y, double heading, double speed, Boolean isCritical, Boolean shift) {
        waypoints.add(new Waypoint(x, y, heading, speed, isCritical, shift));
    }

    /**
     * Contains information to define a point along a desired path
     */
    public static class Waypoint extends Pose {
        // public final Pose point;
        private DoubleSupplier kSpeed;
        protected Boolean isCritical;
        protected Boolean shift=false;

        public Waypoint(double x, double y, double heading, DoubleSupplier speed, Boolean critical) {
            super(x, y, heading);
            this.kSpeed = speed;
            this.isCritical = critical;
        }

        /**
         * Constructor for waypoint
         *
         * @param x        in meters
         * @param y        in meters
         * @param heading  in degrees. Call .r for radians
         * @param speed    in desired speed on a scale of -1 to 1
         * @param critical whether or not the waypoint is critical. Will stop at a
         *                 critical waypoint
         */
        public Waypoint(double x, double y, double heading, double speed, Boolean critical) {
            super(x, y, heading);
            this.kSpeed = () -> speed;
            this.isCritical = critical;
        }

        public Waypoint(double x, double y, double heading, double speed, Boolean critical, Boolean shift) {
            super(x, y, heading);
            this.kSpeed = () -> speed;
            this.isCritical = critical;
            this.shift = shift;
        }

        /**
         * Constructor for waypoint
         *
         * @param x        in meters
         * @param y        in meters
         * @param heading  in degrees. Call .r for radians
         * @param speed    in desired speed on a scale of -1 to 1
         * @param critical whether or not the waypoint is critical. Will stop at a
         *                 critical waypoint
         * @param shift    what gear to shift into for this waypoint
         */

        public Waypoint(Pose pose, DoubleSupplier speed, Boolean critical) {
            this(pose.x, pose.y, pose.heading, speed, critical);
        }

        public Waypoint(Pose pose, double heading, DoubleSupplier speed, Boolean critical) {
            this(pose.x, pose.y, heading, speed, critical);
        }

        public Waypoint(Pose pose, double heading, double speed, Boolean critical) {
            this(pose.x, pose.y, heading, speed, critical);
        }

        public Waypoint(Pose pose, double speed, Boolean critical) {
            this(pose.x, pose.y, pose.heading, speed, critical);
        }

        public Waypoint(double x, double y, double heading, double speed) {
            this(x, y, heading, speed, false);
        }

        public boolean getGear(){
            return shift;
        }

        @Override
        public String toString() {
            return "x: " + x + ", y: " + y + ", heading: " + heading + ", speed: " + kSpeed.getAsDouble();
        }

        public double speed() {
            return kSpeed.getAsDouble();
        }
    }

    public static void main(String[] args) {
        double relativeGoalAngle = -3.2;
        System.out.println((relativeGoalAngle + 2.0*Math.PI) % (2.0*Math.PI) - Math.PI);

    }
}
