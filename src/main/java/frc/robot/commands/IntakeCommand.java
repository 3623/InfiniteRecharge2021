package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase{
    private Intake intake;
    private Spindexer spindexer;

    public IntakeCommand(Intake intake, Spindexer spindexer) {
        this.intake = intake;
        this.spindexer = spindexer;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntaking(true);
        spindexer.setIndexing(true);
    }

    @Override
    public void execute() {
        spindexer.setIndexing(true);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO keep the spinner running until shots
        intake.setIntaking(false);
        CommandScheduler.getInstance().
            schedule(new StartEndCommand(() -> spindexer.setIndexing(true),
                                            () -> spindexer.setIndexing(false),
                                            spindexer).withTimeout(Spindexer.INDEX_TIME));
    }
}