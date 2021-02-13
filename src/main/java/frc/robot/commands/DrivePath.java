/*/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.CubicSplineFollower.Waypoint;
import frc.robot.subsystems.Drivetrain;
import frc.util.Pose;

public class DrivePath extends Command {

    private final Drivetrain dt;
    private Pose startPose;
    private Waypoint[] waypoints;

    public DrivePath(Drivetrain drive, Boolean highSpeed, Pose startPose, Waypoint... waypoints) {
        dt = drive;
        addRequirements(dt);
        this.startPose = startPose;
        this.waypoints = waypoints;
        dt.setShiftMode(highSpeed);
    }

    public DrivePath(Drivetrain drive, Waypoint... waypoints) {
        this(drive, false, null, waypoints);
    }

    @Override
    public void initialize() {
        if (startPose != null) {
            dt.waypointNav.clearWaypoints();
            dt.zeroSensors();
            dt.model.setPosition(startPose);
        }

        for (Waypoint waypoint : waypoints) dt.waypointNav.addWaypoint(waypoint);
        dt.startPathFollowing();
    }

    @Override
    public boolean isFinished() {
        return dt.waypointNav.isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        dt.terribleDrive(0.0, 0.0, false);
    }
}