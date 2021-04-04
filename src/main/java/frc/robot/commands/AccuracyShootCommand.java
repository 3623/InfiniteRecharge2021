package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SpindexerPID;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter.Target;

public class AccuracyShootCommand extends SequentialCommandGroup{
    public AccuracyShootCommand(Shooter shooter, Spindexer spindexer, char zone) {
        addCommands(new Prepare(shooter, spindexer, zone),
                    new WaitCommand(0.5),
                    newFireCommand(shooter, spindexer));
    }

    public static CommandBase newFireCommand(Shooter shooter, Spindexer spindexer) {
        return (new Fire(shooter, spindexer)).withTimeout(spindexer.SHOOT_TIME);
    }

    public static class Prepare extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;
        private char zone;

        public Prepare(Shooter shooter, Spindexer spindexer, char zone) {
            this.spindexer = spindexer;
            this.shooter = shooter;
            this.zone = zone;
            addRequirements(this.shooter, this.spindexer);
        }

        @Override
        public void initialize() {
            super.initialize();
            shooter.seekTarget(Target.GOAL);
            shooter.accuracyShootZoneSet(zone);
            shooter.fire();
            shooter.prepareStart = true;
            spindexer.startReadying();
        }

        @Override
        public boolean isFinished() {
            return shooter.readyToFireCountdown > 10;
        }

        @Override
        public void execute() {
        }

        public void end(boolean interrupted){
            if (!interrupted){
                shooter.prepareDone = true;
            }
            else {
                shooter.disable();
                spindexer.stopSpinning();
            }
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
            spindexer.startShooting();
            shooter.fireStart = true;
        }

        @Override
        public void end(boolean interrupted) {
            spindexer.stopSpinning();
            shooter.disable();
        }
    }
}
