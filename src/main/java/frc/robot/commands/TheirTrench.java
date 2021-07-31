package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.modeling.FieldPositions.LEFT;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class TheirTrench extends SequentialCommandGroup{

    public TheirTrench(Drivetrain drive, Intake intake, Shooter shooter, Spindexer spindexer) {
        super(new DrivePath(drive, false, LEFT.START, LEFT.WAYPOINTS[0])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, LEFT.WAYPOINTS[1])
                  .deadlineWith(new IntakeCommand(intake, spindexer)),
              new ShootCommand.Prepare(shooter, spindexer),
              ShootCommand.newFireCommand(shooter, spindexer),
              new DrivePath(drive, LEFT.WAYPOINTS[2])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, LEFT.WAYPOINTS[3])
                    .deadlineWith(new IntakeCommand(intake, spindexer))
                    .alongWith(new ShootCommand.Prepare(shooter, spindexer)),
              ShootCommand.newFireCommand(shooter, spindexer));
    }

}
