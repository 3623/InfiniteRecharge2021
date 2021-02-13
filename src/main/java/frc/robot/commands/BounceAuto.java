package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.modeling.FieldPositions.BARREL;
import frc.modeling.FieldPositions.BOUNCE;

public class BounceAuto extends SequentialCommandGroup {

    public BounceAuto(Drivetrain drive) {
        super(new DrivePath(drive, false, BOUNCE.START, BOUNCE.WAYPOINTS));
    }

}
