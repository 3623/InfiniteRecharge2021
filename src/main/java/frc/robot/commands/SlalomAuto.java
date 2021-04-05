package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.modeling.FieldPositions.BARREL;
import frc.modeling.FieldPositions.BOUNCE;
import frc.modeling.FieldPositions.SLALOM;

public class SlalomAuto extends SequentialCommandGroup {

    public SlalomAuto(Drivetrain drive) {
        super(new DrivePath(drive, false, SLALOM.START, SLALOM.WAYPOINTS));
    }

}
