package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.modeling.FieldPositions.MIDDLE;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class Middle extends SequentialCommandGroup {

    public Middle(Drivetrain drive, Intake intake, Shooter shooter, Spindexer spindexer) {
        super(new DrivePath(drive, false, MIDDLE.START, MIDDLE.WAYPOINTS[0])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, MIDDLE.WAYPOINTS[1])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new ShootCommand(shooter, spindexer, () -> false),
              new DrivePath(drive, MIDDLE.WAYPOINTS[2])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new ShootCommand(shooter, spindexer, () -> false));
    }

}
