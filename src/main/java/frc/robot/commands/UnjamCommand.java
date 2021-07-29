package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;

public class UnjamCommand extends CommandBase{
    private Spindexer spindexer;

    public UnjamCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.toggleJamClear();
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.toggleJamClear();
    }
}
