/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.modeling.FieldPositions;
import frc.robot.Robot;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Utils;

public class Shooter extends TerribleSubsystem {
    protected final int UPDATE_RATE = 200;

    private Turret turret;
    private Feeder feeder;
    private Hood hood;
    private Flywheel flywheel;
    private Spindexer indexer;

    private static final double AIM_THRESHOLD = 2.0;
    // degrees, +/- that shooter will stil aim for inner port, outside it will shoot at target
    private static final double INNER_PORT_DEPTH = 0.7; // meters
    private static final double INNER_VISIBLE_TRESHOLD = 20.0;

    private Pose robotPose;
    private Pose targetPose = FieldPositions.OUR_GOAL;

    private double targetDistance = 0.0;
    private double targetAngle = 0.0; // global coordinates
    private double innerPortAngle = 0.0;
    private boolean shooting = false;
    public boolean readyToFire = false;

    NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry targetX = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
    NetworkTableEntry targetY = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
    NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
    NetworkTableEntry targets = Lime.getEntry("tv"); // Valid Targets (0 or 1)
    private HttpCamera limeCam;


    private final double LIMELIGHT_ELEVATION_OFFSET = 20.5; // deg
    private final double TARGET_RELATIVE_HEIGHT = 2.0; // meters
    private final double LIMELIGHT_FOV = 54.0;

	private enum ShooterControlState {
        BLIND_AIM, // odometry based turn turret to target
        VISION_TRACKING, // when target has been found
        DISABLED,
    }
    private ShooterControlState controlState = ShooterControlState.DISABLED;

    public enum Target {
        GOAL,
        // LOADING_ZONE, // Maybe these will be used for localization..
        ENEMY_GOAL,
        // ENEMY_LOADING
    }
    private Target target = Target.GOAL;

    public Shooter(Pose odometryModel) {
        setName("Shooter");
        robotPose = odometryModel;
        turret = new Turret();
        hood = new Hood();
        feeder = new Feeder();
        flywheel = new Flywheel();
        indexer = Robot.spindexer;
        limeCam= new HttpCamera("limelight",
                                   "http://limelight.local:5800/stream.mjpg");
        this.updateThreadStart();
        disable();
    }

    /**
     * Begins blind aim and vision tracking for target.
     * It the target is our goal, then will prepare to fire
     * @param target1
     */
    public void seekTarget(Target target1) {
        target = target1;
        if (target == Target.GOAL) {
            shooting = true;
            targetPose = FieldPositions.OUR_GOAL;
        } else if (target == Target.ENEMY_GOAL) {
            targetPose = FieldPositions.THEIR_GOAL;
            shooting = false;
        }
        controlState = ShooterControlState.BLIND_AIM;
        setVision(true);
        enable();
    }

    protected void update() {
        switch (controlState) {
            case BLIND_AIM:
                updateBlind();
                break;
            case VISION_TRACKING:
                updateVision();
                break;
            case DISABLED:
                break;
        }
        if (readyToFire) System.out.println("Ready to fire!");
    }

    private void updateBlind() {
        targetAngle = Geometry.angleBetweenDegrees(robotPose, targetPose);
        targetDistance = Geometry.distance(robotPose, targetPose);
        readyToFire = false;
        setAngle(targetAngle);
        if (Utils.withinThreshold(turret.getMeasurement(), targetAngle, LIMELIGHT_FOV/2.0)
            && targets.getDouble(0.0) == 1) {
                System.out.println("Found vision target, switching to vision tracking");
                controlState = ShooterControlState.VISION_TRACKING;
                updateVision();
            }
        else if (shooting) {
            setDistance(targetDistance);
        }
    }

    private void updateVision() {
        double targetX = this.targetX.getDouble(0.0);
        targetAngle = visionEstimateAngle(targetX);
        targetDistance = visionEstimateDistance(targetY.getDouble(0.0));
        if (Utils.withinThreshold(targetAngle, 0.0, INNER_VISIBLE_TRESHOLD))
            innerPortAngle = Math.atan(targetDistance * Math.sin(Math.toRadians(targetAngle)) /
                                    (targetDistance * Math.cos(Math.toRadians(targetAngle)) + INNER_PORT_DEPTH));
        else innerPortAngle = targetAngle;
        if (shooting) {
            setAngle(innerPortAngle);
            setDistance(targetDistance);
            readyToFire = Utils.withinThreshold(targetX, 0.0, AIM_THRESHOLD);
            readyToFire &= flywheel.isAtSpeed();
            readyToFire &= indexer.isReady(); // TODO check hood;
        }
        // Decide when to localize odometry if there is a vision target
        // Might be a time for filters!
    }

    private double visionEstimateDistance(double targetY) {
        double realElevation = targetY + LIMELIGHT_ELEVATION_OFFSET;
        return TARGET_RELATIVE_HEIGHT / Math.sin(Math.toRadians(realElevation));
    }

    private double visionEstimateAngle(double targetX) {
        return targetX + robotPose.heading + turret.getMeasurement();
    }

    /**
     * Set the turret pid setpoint, adjusted for robot rotation
     *
     * @param angle - in global coordinations
     */
    public void setAngle(double angle) { // This is only public for testing
        turret.setSetpoint(Geometry.limitAngleDegrees(angle - robotPose.heading));
    }

    /**
     * Controls the flywheel and hood
     * TODO make this do something smart
     * @param angle
     */
    public void setDistance(double distance) {
        flywheel.setSpeed(4200.0);
        // hood.setSetpoint(0.0);
    }

    /**
     * Sets limelight leds and camera mode
     */
    public void setVision(boolean on) {
        NetworkTable lm = NetworkTableInstance.getDefault().getTable("limelight");
        if (on) {
            lm.getEntry("ledMode").setNumber(3);
            lm.getEntry("camMode").setNumber(0);
        } else {
            lm.getEntry("ledMode").setNumber(1);
            lm.getEntry("camMode").setNumber(1);
        }
    }

    /**
     * Called whenever robot is disabled and after all balls are shot
     */
    public void disable() {
        System.out.println("Shooter disabled");
        controlState = ShooterControlState.DISABLED;
        setVision(false);
        shooting = false;
        readyToFire = false;
        turret.disable();
        hood.disable();
        flywheel.disable();
        feeder.stop();
    }

    public void enable() {
        turret.enable();
        hood.enable();
    }

    public void fire() {
        feeder.run(1.0);
    }

    public void zeroSensors() {
        turret.zero();
        hood.zero();
    }

    @Override
    public void periodic() {
        super.periodic();
        display("Distance", targetDistance);
        display("Angle", targetAngle);
        display("Inner port", innerPortAngle);
        turret.monitor();
        hood.monitor();
    }

}
