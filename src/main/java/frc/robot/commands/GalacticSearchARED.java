package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.modeling.FieldPositions.BARREL;
import frc.modeling.FieldPositions.BOUNCE;
import frc.modeling.FieldPositions.PICKUP;

public class GalacticSearchARED extends SequentialCommandGroup {

    public GalacticSearchARED(Drivetrain drive) {
        super(new DrivePath(drive, false, PICKUP.START, PICKUP.A_RED));
    }

}
