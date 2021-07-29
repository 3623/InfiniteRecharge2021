package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;

public class SimpleShoot extends SequentialCommandGroup{

    public SimpleShoot(Drivetrain drive, Intake intake, Shooter shooter, Spindexer spindexer) {
        super(new ShootCommand(shooter, spindexer, () -> false).withTimeout(13.0),
              new RunCommand(() -> drive.terribleDrive(0.5, 0.0, false), drive).withTimeout(2.0));
    }

}
