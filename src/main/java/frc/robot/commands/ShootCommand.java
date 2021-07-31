package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class ShootCommand extends SequentialCommandGroup{

    public ShootCommand(Shooter shooter, Spindexer spindexer, BooleanSupplier prepareHold, XboxController... users) {
        addCommands(new Prepare(shooter, spindexer, prepareHold, users),
                    new WaitCommand(0.1),
                    newFireCommand(shooter, spindexer, users));
    }

    public static CommandBase newFireCommand(Shooter shooter, Spindexer spindexer, XboxController... users) {
        return (new Fire(shooter, spindexer, users)).withTimeout(Spindexer.SHOOT_TIME);
    }

    protected static void setRumble(double value, XboxController[] users) {
        for (XboxController user : users) {
            user.setRumble(RumbleType.kLeftRumble, value);
            user.setRumble(RumbleType.kRightRumble, value);
        }
    }

    public static class Prepare extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;
        private BooleanSupplier prepareHold;
        private XboxController[] users;

        public Prepare(Shooter shooter, Spindexer spindexer, BooleanSupplier prepareHold, XboxController... users) {
            this.spindexer = spindexer;
            this.shooter = shooter;
            addRequirements(this.shooter, this.spindexer);
            this.prepareHold = prepareHold;
            this.users = users;
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
            setRumble(0.2, users);
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
            setRumble(0.0, users);
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
            System.out.println("Firing");
            spindexer.startShooting();
            shooter.fire();
            setRumble(0.8, users);
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.stopSpinning();
            shooter.disable();
            setRumble(0.0, users);
        }
    }
}
