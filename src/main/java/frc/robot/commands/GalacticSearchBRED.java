package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.modeling.FieldPositions.BARREL;
import frc.modeling.FieldPositions.BOUNCE;
import frc.modeling.FieldPositions.PICKUP;

public class GalacticSearchBRED extends SequentialCommandGroup {

    public GalacticSearchBRED(Drivetrain drive, Intake intake) {
        super(new SearchIntake(intake),
        new DrivePath(drive, false, PICKUP.START, PICKUP.B_RED));
    }

}
