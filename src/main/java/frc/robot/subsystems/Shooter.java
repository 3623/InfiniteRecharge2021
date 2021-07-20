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
import frc.robot.Robot;
import frc.util.Pose;
import frc.util.Utils;

public class Shooter extends TerribleSubsystem {
    protected final int UPDATE_RATE = 200;

    private Turret turret;
    private Feeder feeder;
    private Hood hood;
    private Flywheel flywheel;

    /** degrees */
    private static final double AIM_THRESHOLD = 5.0;

    /** meters */
    private static final double DEFAULT_SHOOTER_DISTANCE = 3.0;

    private Pose robotPose;

    private double targetDistance = 0.0;
    /** global reference */
    private double targetAngle = 0.0;
    private boolean readyToFire = false;

    private double hoodTrim = 0.0;
    private double turretTrim = 0.0;
    /** kRPMS */
    private double rpmTrim = 0.0;

    NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry targetX = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
    NetworkTableEntry targetY = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
    NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
    NetworkTableEntry targets = Lime.getEntry("tv"); // Valid Targets (0 or 1)
    private HttpCamera limeCam; // We need this for it to work!

    private boolean manualOverride = false;

    private final double LIMELIGHT_ELEVATION_OFFSET = 20.5; // deg
    private final double TARGET_RELATIVE_HEIGHT = 2.0; // meters
    private final double LIMELIGHT_FOV = 54.0;

	private enum ShooterControlState {
        /** look forward for target or manual control */
        BLIND_AIM,
        /** when target has been found */
        VISION_TRACKING,
        /** robot disabled or after shooting*/
        DISABLED,
    }
    private ShooterControlState controlState = ShooterControlState.DISABLED;

    public Shooter(Pose odometryModel) {
        setName("Shooter");
        robotPose = odometryModel;
        turret = new Turret();
        hood = new Hood();
        feeder = new Feeder();
        flywheel = new Flywheel();
        limeCam = new HttpCamera("limelight",
                                   "http://limelight.local:5800/stream.mjpg");
        this.updateThreadStart();
        disable();
    }

    /**
     * Begins blind aim and vision tracking for target.
     * It the target is our goal, then will prepare to fire
     * @param target1
     */
    public void prepare() {
        controlState = ShooterControlState.BLIND_AIM;
        setVision(true);
        enable();
        this.fire();
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
        // = Geometry.distance(robotPose, targetPose);
        readyToFire = false;
        targetAngle = 0.0;
        targetDistance = DEFAULT_SHOOTER_DISTANCE;
        if (manualOverride) {
            // Use the turret trim (for manual control of aiming)
            targetAngle += turretTrim;
            // Stay in update blind
        } else if (Utils.withinThreshold(turret.getMeasurement() + robotPose.heading,
                                  targetAngle,
                                  LIMELIGHT_FOV*0.7)
            && targets.getDouble(0.0) == 1) {
                System.out.println("Found vision target, switching to vision tracking");
                controlState = ShooterControlState.VISION_TRACKING;
                updateVision();
        }
        setAngle(targetAngle);
        setDistance(targetDistance);
    }

    private void updateVision() {
        // We should never be in manual override and here at the same time!
        double targetX = this.targetX.getDouble(0.0);
        targetAngle = limeToGlobalAngle(targetX);
        targetDistance = visionEstimateDistance(targetY.getDouble(0.0));

        setAngle(targetAngle);
        setDistance(targetDistance);

        // Decide when to localize odometry if there is a vision target
        // Might be a time for filters!
    }

    /**
     *
     * @param targetY
     * @return distance to target in meters
     */
    private double visionEstimateDistance(double targetY) {
        double realElevation = targetY + LIMELIGHT_ELEVATION_OFFSET;
        return TARGET_RELATIVE_HEIGHT / Math.sin(Math.toRadians(realElevation));
    }


    /**
     *
     * @param targetHoriz limelight horizontal location for the center of the target in degrees
     * @return global heading to target in degrees
     */
    private double limeToGlobalAngle(double targetHoriz) {
        return targetHoriz + robotPose.heading + turret.getMeasurement();
    }

    // public void accuracyShootZoneSet(char zone){
    //     this.zone = zone;
    //     if (zone == 'g') {
    //         targetSpeed = 8250.0;
    //         targetHood = 0.0;
    //     }
    //     else if (zone == 'y') {
    //         targetSpeed = 7750.0;
    //         targetHood = 22.5;
    //     }
    //     else if (zone == 'b') {
    //         targetSpeed = 7500.0;
    //         targetHood =  25.0;
    //     }
    //     else if (zone == 'r') {
    //         targetSpeed = 7750.0;
    //         targetHood = 17.5;
    //     }
    //     else {
    //         targetSpeed = 4000;
    //         targetHood = 0.0;
    //     }
    // }

    /**
     * Set the turret pid setpoint, adjusted for robot rotation
     * Should never be public. Use manualTurret because that checks for override
     *
     * @param angle - in global reference
     */
    private void setAngle(double angle) {
        // TODO this is limited in Turret (check?)
        turret.setSetpoint(angle - robotPose.heading);
    }

    /**
     * Controls the flywheel and hood
     * @param angle
     */
    public void setDistance(double dist) {
        dist = Utils.limit(dist, 7, 0);
        // TODO tune this
        double angle = -18.6 + (dist*21.2) - (dist*dist*2.54);
        double rpm = 8.73 - (dist*0.528) + (dist*dist*0.0599);
        flywheel.setSpeed((rpm + rpmTrim) * 1000.0);
        hood.setSetpoint(angle + hoodTrim);
    }

    /**
     *
     * @param on true sets led on and liemelight to vision mode, else off for webcam
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
     * Called whenever robot is disabled or after all balls are shot
     */
    public void disable() {
        controlState = ShooterControlState.DISABLED;
        setVision(false);
        readyToFire = false;
        turret.disable();
        // hood.disable();
        // TODO hood needs to stow (check)
        hood.setSetpoint(0.0);
        flywheel.disable();
        feeder.stop();
    }

    // TODO is this needed?
    public void enable() {
        turret.enable();
        //hood.enable();
    }

    private void fire() {
        feeder.run(1.0);
    }

    public boolean isReadyToFire() {
        readyToFire = (controlState == ShooterControlState.VISION_TRACKING || manualOverride);
        readyToFire &= Utils.withinThreshold(targetAngle, 0.0, AIM_THRESHOLD);
        readyToFire &= flywheel.isAtSpeed();
        readyToFire &= Robot.spindexer.isReady();
        readyToFire &= hood.isReady();
        return readyToFire;
    }

    public void zeroSensors() {
        turret.zero();
    }

    public void manualTurret(double angle){
        if (manualOverride) setAngle(angle);
    }

    public void trimHood(double delta){
        hoodTrim += delta;
    }

    public void trimTurret(double delta) {
        turretTrim += delta;
    }

    /**
     *
     * @param delta - how much to change the RPM, in kRPMS
     */
    public void trimRPM(double delta) {
        rpmTrim += delta;
    }

    @Override
    public void periodic() {
        super.periodic();
        display("Distance", targetDistance);
        display("Angle", targetAngle);
        display("Ready to Fire Overall", readyToFire);
        display("At Speed", flywheel.isAtSpeed());
        display("Manual Override", manualOverride);
        display("State", controlState.toString());
        turret.monitor();
        hood.monitor();
        Robot.spindexer.monitor();
    }

}
