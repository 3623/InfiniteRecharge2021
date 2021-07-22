package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

// TODO rumble feedback (check)
public class ShootCommand extends SequentialCommandGroup{

    public ShootCommand(Shooter shooter, Spindexer spindexer, XboxController... users) {
        addCommands(new Prepare(shooter, spindexer, users),
                    new WaitCommand(0.5),
                    newFireCommand(shooter, spindexer, users));
    }

    public static CommandBase newFireCommand(Shooter shooter, Spindexer spindexer, XboxController... users) {
        return (new Fire(shooter, spindexer, users)).withTimeout(Spindexer.SHOOT_TIME);
    }

    public static class Prepare extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;
        private XboxController[] users;

        public Prepare(Shooter shooter, Spindexer spindexer, XboxController... users) {
            this.spindexer = spindexer;
            this.shooter = shooter;
            addRequirements(this.shooter, this.spindexer);
            this.users = users;
        }

        @Override
        public void initialize() {
            super.initialize();
            shooter.prepare();
            spindexer.startReadying();
            for (XboxController user : users) {
                user.setRumble(RumbleType.kLeftRumble, 0.2);
                user.setRumble(RumbleType.kRightRumble, 0.2);
            }
        }

        @Override
        public boolean isFinished() {
            return shooter.isReadyToFire();
        }

        @Override
        public void execute() {
        }

        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.disable();
                spindexer.stopSpinning();
            }
            for (XboxController user : users) {
                user.setRumble(RumbleType.kLeftRumble, 0.0);
                user.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }
    }

    private static class Fire extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;
        private XboxController[] users;

        private Fire(Shooter shooter, Spindexer spindexer, XboxController... users) {
            this.shooter = shooter;
            this.spindexer = spindexer;
            addRequirements(shooter, spindexer);
            this.users = users;
        }

        @Override
        public void initialize() {
            spindexer.startShooting();
            for (XboxController user : users) {
                user.setRumble(RumbleType.kLeftRumble, 0.5);
                user.setRumble(RumbleType.kRightRumble, 0.5);
            }
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.stopSpinning();
            shooter.disable();
            for (XboxController user : users) {
                user.setRumble(RumbleType.kLeftRumble, 0.0);
                user.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }
    }
}
