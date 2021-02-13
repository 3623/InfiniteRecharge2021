package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.modeling.FieldPositions.RIGHT;

public class OurTrench extends SequentialCommandGroup{

    public OurTrench(Drivetrain drive, Intake intake, Shooter shooter, Spindexer spindexer) {
        super(new DrivePath(drive, false, RIGHT.START, RIGHT.WAYPOINTS[0])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, RIGHT.WAYPOINTS[1])
                    .alongWith(new ShootCommand.Prepare(shooter, spindexer)),
              ShootCommand.newFireCommand(shooter, spindexer),
              new DrivePath(drive, RIGHT.WAYPOINTS[2], RIGHT.WAYPOINTS[3])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, RIGHT.WAYPOINTS[4])
                    .deadlineWith(new IntakeCommand(intake, spindexer))
                    .alongWith(new ShootCommand.Prepare(shooter, spindexer)),
              ShootCommand.newFireCommand(shooter, spindexer));
    }

}
