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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class CubicSplineFollower {
    private static final double WHEEL_BASE = DrivetrainModel.WHEEL_BASE;
    private static final double UPDATE_RATE = 200.0;

    private LinkedList<Waypoint> waypoints;
    private Waypoint curWaypoint = null;

    public Boolean isFinished = false;

    private static final double kMaxAccel = 0.2; // m/s^2 * 200
    private static final double kNextSpeedFactor = 0.02; // 0.0 - 1.0
    private static final double kTurnMultiplier = 1.0; // should be close to 1
    private static final double kMaxAngularDiffFactor = 3.0; // m/s * 2
    private static final double kSlowdownRadius = 1.0; // m
    private static final double kMinApproachSpeedCritical = 0.1; // %
    private static final double kRadiusCritical = 0.1;; // m
    private static final double kScaleRadiusPath = 0.2; // constant
    private double kRadiusPath = 0.0; // this updates dynamically
    // deg, keeping this because this dictates when the robot switches
    private static final double kAngularErrorPath = 20.0;
    private static final double kMaxSplineAngle = Math.PI * 0.3;

    private DrivetrainModel drivetrainState;

    private double distToWaypoint;
    private double DT_MAX_SPEED;
    /** % speed */
    private double ffSpeed;
    private boolean onBoard;

    private static final Boolean debug = false;
    protected static final Boolean DEFAULT_GEAR = false;

    public CubicSplineFollower(DrivetrainModel drivetrain) {
        drivetrainState = drivetrain;

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
        // Check for finished case
        boolean finished = waypoints.isEmpty() && curWaypoint == null;
        if (finished && !isFinished) System.out.println("Finished Path Following");
        isFinished = finished;
        if (isFinished) return new Tuple(0.0, 0.0);

        // New waypoint
        if (curWaypoint == null) curWaypoint = waypoints.pollFirst();
        distToWaypoint = Geometry.distance(robotPose, curWaypoint);
        DT_MAX_SPEED = drivetrainState.getTopSpeed();
        ffSpeed = curWaypoint.speed();

        boolean nextWaypoint = false;
        /* important to be at exactly */
        if (curWaypoint.isCritical) {
            // speed reduces as distance gets smaller
            if (distToWaypoint < Math.abs(ffSpeed) * kSlowdownRadius) {
                if (debug) System.out.println("Slowing down");
                ffSpeed = Math.copySign(distToWaypoint / kSlowdownRadius, ffSpeed);
                // TODO this might not be necessary
                if (Math.abs(ffSpeed) < kMinApproachSpeedCritical)
                    ffSpeed = Math.copySign(kMinApproachSpeedCritical, ffSpeed);
            }
            // at point and heading, we're done
            if (distToWaypoint < kRadiusCritical || isFinished)
                nextWaypoint = true;
        }
        /* Path waypoint, just check radius */
        else if (distToWaypoint < kRadiusPath
                && Utils.withinThreshold(robotPose.heading, curWaypoint.heading, kAngularErrorPath))
                // at non-critical waypoint
                nextWaypoint = true;

        if (nextWaypoint) {
            System.out.println("At Waypoint: " + curWaypoint.toString());
            curWaypoint = waypoints.pollFirst();
            return updatePursuit(robotPose);
        } else if (debug)
            System.out.println("Seeking: " + curWaypoint.toString());

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
        display("ffSpeed", ffSpeed, true);
        Tuple pathCoefficients = getPathGeometry(robotPose, curWaypoint);
        double a = pathCoefficients.left;
        double b = pathCoefficients.right;
        // Without next speed factor, when the robot is stopped this would be 0
        double nextSpeed = ((DT_MAX_SPEED * ffSpeed) * kNextSpeedFactor) +
                            (robotPose.velocity * (1.0-kNextSpeedFactor));
        // Next position along spline the robot is trying to reach
        // TODO might want to find a way to more accurately calculate deltaX
        double deltaX = nextSpeed / UPDATE_RATE;

        // Dont calculate speed for the spline in the wrong direction
        // Just let the robot sit until then
        if (Math.signum(deltaX) != Math.signum(ffSpeed))
            deltaX = 0.0;
        display("deltaX Raw", deltaX, true);

        if (deltaX != 0.0) {
            double y2 = (a * deltaX * deltaX * deltaX) + (b * deltaX * deltaX);
            double hypot = Geometry.hypotenuse(deltaX, y2);
            double ratio = Math.abs(deltaX / hypot);
            deltaX *= ratio;
        }
        display("deltaX Scaled", deltaX, true);

        kRadiusPath = Math.abs(deltaX) * UPDATE_RATE * kScaleRadiusPath;
        display("radius", kRadiusPath, true);
        double dx2 = (3.0 * a * deltaX * deltaX) + (2.0 * b * deltaX);
        double relFFAngle = Math.atan(dx2);
        display("relFFAngle", relFFAngle, true);
        double omega = relFFAngle * UPDATE_RATE;
        display("omega", omega, true);

        // Convert from derivative to angle

        double desiredSpeed = ffSpeed * DT_MAX_SPEED;
        display("robot velo", robotPose.velocity, true);
        display("Raw desiredSpeed", desiredSpeed, true);
        // Limit output per acceleration
        if (desiredSpeed - robotPose.velocity > kMaxAccel)
            desiredSpeed = robotPose.velocity + kMaxAccel;
        else if (desiredSpeed - robotPose.velocity < -kMaxAccel)
            desiredSpeed = robotPose.velocity - kMaxAccel;
        display("Accel desiredSpeed", desiredSpeed, true);

        double maxTurn = kMaxAngularDiffFactor * Math.abs(desiredSpeed);
        display("Max turn", maxTurn, true);

        double lrSpeedDifference = omega * WHEEL_BASE * kTurnMultiplier;
        lrSpeedDifference = Utils.limit(lrSpeedDifference, maxTurn, -maxTurn);
        // Limit output to allow turning
        if (desiredSpeed + Math.abs(lrSpeedDifference) > DT_MAX_SPEED)
            desiredSpeed = DT_MAX_SPEED - Math.abs(lrSpeedDifference);
        else if (desiredSpeed - Math.abs(lrSpeedDifference) < -DT_MAX_SPEED)
            desiredSpeed = -DT_MAX_SPEED + Math.abs(lrSpeedDifference);
        display("Turn desired speed", desiredSpeed, true);

        double leftSpeed = desiredSpeed - (lrSpeedDifference / 2);
        double rightSpeed = desiredSpeed + (lrSpeedDifference / 2);
        display("lrSpeedDifference", lrSpeedDifference, true);
        display("leftSpeed", leftSpeed, true);
        display("rightSpeed", rightSpeed, true);
        return new Tuple(leftSpeed, rightSpeed);
    }

    /**
     * Calculates the rel angles and distances from the current robot position
     * to the desired goal point.
     *
     * @param startPoint the start position of the robot, if using dynamic path
     *                   generation, this should be the robot position
     * @param goalPoint  the goal position of the path to be calculated
     * @return a tuple of path coefficients a and b respectively for a cubic spline
     */
    private Tuple getPathGeometry(Pose startPoint, Pose goalPoint) {
        double straightPathAngle = Math.atan2(goalPoint.x - startPoint.x, goalPoint.y - startPoint.y);
        double relAngle = startPoint.r - straightPathAngle;

        double relOpposDist = distToWaypoint * Math.sin(relAngle);
        double relAdjacDist = distToWaypoint * Math.cos(relAngle);
        double relGoalAngle = startPoint.r - goalPoint.r;
        relGoalAngle = Geometry.limitAngleRad(relGoalAngle);
        relGoalAngle = Utils.limit(relGoalAngle, kMaxSplineAngle, -kMaxSplineAngle);
        double relGoalDeriv = Math.tan(relGoalAngle);
        if (debug) {
            System.out.println(relAdjacDist + " " + relOpposDist + " " + relGoalDeriv);
        }
        return generateSpline(relAdjacDist, relOpposDist, relGoalDeriv);
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

    private void display(String name, double value, boolean trace) {
        if (!trace || debug) {
            if (onBoard)
                SmartDashboard.putNumber("Path Following/" + name, value);
            else
                System.out.println(name + ": " + value);
        }
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
        /** Double supplier so that it can be used with assisted driving */
        private DoubleSupplier kSpeed;
        /** true is waypoint should be stopped at, by default false */
        public final Boolean isCritical;
        /** true if high gear */
        public final Boolean shift;

        /**
         * Constructor for waypoint
         *
         * @param x        in meters
         * @param y        in meters
         * @param heading  in degrees. Call .r for radians
         * @param speed    in desired speed on a scale of -1 to 1
         * @param critical whether or not the waypoint is critical. Will stop at a
         *                 critical waypoint
         * @param shift    high gear if true
         *
         */
        public Waypoint(double x, double y, double heading, DoubleSupplier speed, Boolean critical, Boolean shift) {
            super(x, y, heading);
            this.kSpeed = speed;
            this.isCritical = critical;
            this.shift = shift;
        }

        public Waypoint(double x, double y, double heading, DoubleSupplier speed, Boolean critical) {
            this(x, y, heading, speed, critical, DEFAULT_GEAR);
        }

        public Waypoint(double x, double y, double heading, double speed, Boolean critical, Boolean shift) {
            this(x, y, heading, () -> speed, critical, shift);
        }

        public Waypoint(double x, double y, double heading, double speed, Boolean critical) {
            this(x, y, heading, speed, critical, DEFAULT_GEAR);
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


        @Override
        public String toString() {
            return "x: " + x + ", y: " + y + ", heading: " + heading + ", speed: " + kSpeed.getAsDouble();
        }

        public double speed() {
            return kSpeed.getAsDouble();
        }
    }

    public static void main(String[] args) {
    }
}
