package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.SpindexerPID;
import frc.robot.subsystems.Intake;

public class SearchIntake extends CommandBase{
    private Intake intake;

    public SearchIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntaking(true);
    }

}