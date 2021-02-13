package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Target;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(Shooter shooter, Spindexer spindexer) {
        addCommands(new Prepare(shooter, spindexer),
                    newFireCommand(shooter, spindexer));
    }

    public static CommandBase newFireCommand(Shooter shooter, Spindexer spindexer) {
        return (new Fire(shooter, spindexer)).withTimeout(Spindexer.SHOOT_TIME);
    }

    public static class Prepare extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;

        public Prepare(Shooter shooter, Spindexer spindexer) {
            this.spindexer = spindexer;
            this.shooter = shooter;
            addRequirements(shooter);
        }

        @Override
        public void initialize() {
            super.initialize();
            shooter.seekTarget(Target.GOAL);
            spindexer.setShooting(true);
        }

        @Override
        public boolean isFinished() {
            return shooter.readyToFire;
        }

        @Override
        public void execute() {
        }
    }

    private static class Fire extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;

        private Fire(Shooter shooter, Spindexer spindexer) {
            this.shooter = shooter;
            this.spindexer = spindexer;
            addRequirements(shooter);
        }

        @Override
        public void initialize() {
            shooter.fire();
            spindexer.setShooting(true);
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.setShooting(false);
            spindexer.setIndexing(false);
            shooter.disable();
        }
    }
}
