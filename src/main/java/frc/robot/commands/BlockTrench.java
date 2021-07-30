package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.modeling.FieldPositions.BLOCK;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class BlockTrench extends SequentialCommandGroup{

    public BlockTrench(Drivetrain drive, Intake intake, Shooter shooter, Spindexer spindexer) {
        super(new DrivePath(drive, false, BLOCK.START, BLOCK.WAYPOINTS[0])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, BLOCK.WAYPOINTS[1], BLOCK.WAYPOINTS[2])
                    .alongWith(new ShootCommand.Prepare(shooter, spindexer)),
              ShootCommand.newFireCommand(shooter, spindexer),
              new DrivePath(drive, BLOCK.WAYPOINTS[3])
                    .deadlineWith(new IntakeCommand(intake, spindexer)),
              new DrivePath(drive, BLOCK.WAYPOINTS[4])
                    .alongWith(new ShootCommand.Prepare(shooter, spindexer)),
              ShootCommand.newFireCommand(shooter, spindexer));
    }
}
