package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.modeling.FieldPositions.BARREL;
import frc.modeling.FieldPositions.BOUNCE;
import frc.modeling.FieldPositions.PICKUP;

public class GalacticSearchBBLUE extends SequentialCommandGroup {

    public GalacticSearchBBLUE(Drivetrain drive, Intake intake, Spindexer spin) {
        super(new ParallelCommandGroup(
                            new SearchIntake(intake, spin),
                new DrivePath(drive, false, PICKUP.START, PICKUP.B_BLUE)));
    }

}
