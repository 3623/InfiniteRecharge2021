package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.modeling.FieldPositions.BARREL;

public class BarrelAuto extends SequentialCommandGroup{

    public BarrelAuto(Drivetrain drive) {
        super(new DrivePath(drive, false, BARREL.START, BARREL.WAYPOINTS));
    }

}
