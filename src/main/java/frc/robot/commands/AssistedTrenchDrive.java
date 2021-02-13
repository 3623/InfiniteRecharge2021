/*/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.controls.CubicSplineFollower.Waypoint;
import frc.modeling.FieldPositions;
import frc.robot.subsystems.Drivetrain;

// TODO make this use DrivePath!
public class AssistedTrenchDrive extends CommandBase {

    private Drivetrain dt;
    private DoubleSupplier speed;

    public AssistedTrenchDrive(Drivetrain drive, DoubleSupplier speed) {
        dt = drive;
        this.speed = speed;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        dt.waypointNav.clearWaypoints();
        double y = dt.model.center.y;
        if (y < FieldPositions.TRENCH_CUTOFF)
            dt.waypointNav.addWaypoint(new Waypoint(FieldPositions.PRE_TRENCH_CYCLE, speed, false));
        double heading = 0.0;
        double speed = 1.0;
        if (y > FieldPositions.FAR_CUTOFF) {
            heading = 90.0;
            if (dt.model.center.heading < 0) {
                heading = -90.0;
                speed = -1.0;
            }
        }
        dt.waypointNav.addWaypoint(new Waypoint(FieldPositions.TRENCH_CYCLE_SHOT, heading, speed, true));
        dt.setShiftMode(true);
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