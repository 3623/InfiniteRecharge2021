package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

// TODO rumble feedback (check)
public class ShootCommand extends SequentialCommandGroup{
    private static XboxController[] users;

    public ShootCommand(Shooter shooter, Spindexer spindexer, BooleanSupplier prepareHold, XboxController... users) {
        addCommands(new Prepare(shooter, spindexer, prepareHold),
                    new WaitCommand(0.1),
                    newFireCommand(shooter, spindexer));
        ShootCommand.users = users;
    }

    public static CommandBase newFireCommand(Shooter shooter, Spindexer spindexer) {
        return (new Fire(shooter, spindexer)).withTimeout(Spindexer.SHOOT_TIME);
    }

    protected static void setRumble(double value) {
        for (XboxController user : users) {
            user.setRumble(RumbleType.kLeftRumble, value);
            user.setRumble(RumbleType.kRightRumble, value);
        }
    }

    public static class Prepare extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;
        private BooleanSupplier prepareHold;

        public Prepare(Shooter shooter, Spindexer spindexer, BooleanSupplier prepareHold) {
            this.spindexer = spindexer;
            this.shooter = shooter;
            addRequirements(this.shooter, this.spindexer);
            this.prepareHold = prepareHold;
        }

        public Prepare(Shooter shooter, Spindexer spindexer) {
            this(shooter, spindexer, () -> false);
        }

        @Override
        public void initialize() {
            System.out.println("Preparing");
            super.initialize();
            shooter.prepare();
            spindexer.startReadying();
            setRumble(0.1);
        }

        @Override
        public boolean isFinished() {
            return shooter.isReadyToFire() && (!prepareHold.getAsBoolean());
        }

        @Override
        public void execute() {
        }

        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.disable();
                spindexer.stopSpinning();
            }
            setRumble(0.0);
        }
    }

    private static class Fire extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;

        private Fire(Shooter shooter, Spindexer spindexer) {
            this.shooter = shooter;
            this.spindexer = spindexer;
            addRequirements(shooter, spindexer);
        }

        @Override
        public void initialize() {
            System.out.println("Firing");
            spindexer.startShooting();
            setRumble(0.4);
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.stopSpinning();
            shooter.disable();
            setRumble(0.0);
        }
    }
}
