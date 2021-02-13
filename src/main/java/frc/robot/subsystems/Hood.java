package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Hood extends PIDSubsystem {
    WPI_VictorSPX motor;

    private Encoder encoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = 1.0 / TICKS_PER_ENCODER_REV * 24.0 / 40.5 * 45.0;

    private double MAX_GOAL = 30.0;
    private double MIN_GOAL = 0.0;

    private static final double kP = 1.3 / 20.0;
    private static final double kI = kP / 500.0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = .25;

    public Hood() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);

        motor = new WPI_VictorSPX(Constants.Shooter.SHOOTER_HOOD_MOTOR_SPX);
        encoder = new Encoder(2, 3);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void monitor() {
        SmartDashboard.putNumber("Shooter/Hood/Position", this.getMeasurement() + 45);
        SmartDashboard.putNumber("Shooter/Hood/Output", (int) motor.getMotorOutputPercent() * 100);
        SmartDashboard.putNumber("Shooter/Hood/Error", getController().getPositionError());
        SmartDashboard.putNumber("Shooter/Hood/Setpoint", getController().getSetpoint());
    }

    public void setPosition(double position) {
        if (position > MAX_GOAL) {
            position = MAX_GOAL;
        } else if (position < MIN_GOAL) {
            position = MIN_GOAL;
        }
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
