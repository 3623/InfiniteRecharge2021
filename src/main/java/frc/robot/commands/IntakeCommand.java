package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        spindexer.startIndex();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntaking(false);
        spindexer.setStandby();
    }
}