package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.SpindexerPID;
import frc.robot.subsystems.Intake;

public class SearchIntake extends CommandBase{
    private Intake intake;
    private Spindexer spin;

    public SearchIntake(Intake intake, Spindexer spin) {
        this.intake = intake;
        this.spin = spin;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntaking(true);
        spin.startIndex();
    }

    public boolean isFinished(){
        return true;
    }
}