package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

// TODO Update system to be a drop in replacement for Spindexer to run on PID

public class SpindexerPID extends PIDSubsystem {
    WPI_VictorSPX motor;

    private Encoder encoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = ((1.0 / TICKS_PER_ENCODER_REV)*(1/(400/24))/5); 
    
    private static final double kP = 1.3 / 20.0;
    private static final double kI = kP / 500.0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = .25;

    public SpindexerPID() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);

        motor = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
        encoder = new Encoder(4, 5);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        
    }

    public void monitor() {
        
    }

    public void setPosition(double position) {
        setSetpoint(position);
    }

    public void setRelative(double offSet) {
        setPosition(this.getMeasurement() + offSet);
        if (offSet > 0.01){
            System.out.println("I have an offset of: " + offSet);
        }
    }

    public void zero() {
        encoder.reset();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor.set(ControlMode.PercentOutput, output);
    }

    public void runWithOutput(double output) {
        motor.set(output);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getDistance();
    }
}
