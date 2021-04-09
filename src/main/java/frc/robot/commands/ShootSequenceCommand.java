package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SpindexerPID;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter.Target;

public class ShootSequenceCommand extends SequentialCommandGroup{
    public ShootSequenceCommand(Shooter shooter, Spindexer spindexer) {
        addCommands(new Prepare(shooter, spindexer),
                    newFireSequence(shooter, spindexer));
    }

    public static SequentialCommandGroup newFireSequence(Shooter shooter, Spindexer spindexer){
        return new SequentialCommandGroup(new fireOne(shooter, spindexer),
                                            new fireOne(shooter, spindexer),
                                            new fireOne(shooter, spindexer),
                                            new fireOne(shooter, spindexer),
                                            new fireOne(shooter, spindexer));
    }

    

    public static class Prepare extends CommandBase {
        private Shooter shooter;
        private Spindexer spindexer;

        public Prepare(Shooter shooter, Spindexer spindexer) {
            this.spindexer = spindexer;
            this.shooter = shooter;
            addRequirements(this.shooter, this.spindexer);
        }

        @Override
        public void initialize() {
            super.initialize();
            shooter.seekTarget(Target.GOAL);
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

    public static class fireOne extends CommandBase{
        private Shooter shooter;
        private Spindexer spindexer;

        public fireOne(Shooter shooter, Spindexer spindexer){
            this.shooter = shooter;
            this.spindexer = spindexer;
            addRequirements(this.shooter, this.spindexer);
        }

        public void initialize(){
            super.initialize();
            spindexer.startMoveOne();
        }

        public boolean isFinished(){
            return (spindexer.isReady() && (spindexer.getMode() == 0) && shooter.readyToFireCountdown > 10);
        }
    }
}
