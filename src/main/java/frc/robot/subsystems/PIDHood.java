package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.util.Utils;

public class PIDHood extends PIDSubsystem {
    // WPI_VictorSPX motor;
    PWM motor;

    private Encoder encoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = 1.0 / TICKS_PER_ENCODER_REV * 24.0 / 40.5 * 45.0;

    private double MAX_GOAL = 35.0;
    private double MIN_GOAL = 0.0;

    private static final double kP = 0.2;
    private static final double kI = 0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = .25;

    private double lastOutput = 0.0;

    public PIDHood() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);

        // motor = new WPI_VictorSPX(Constants.Shooter.SHOOTER_HOOD_MOTOR_SPX);
        motor = new PWM(0);
        double boundCenter = motor.getRawBounds().center;
        //motor.setBounds(boundCenter+0.5, boundCenter+0.02, boundCenter, boundCenter-0.02, boundCenter-0.5);
        encoder = new Encoder(5, 4);
        // motor.setNeutralMode(NeutralMode.Brake);
        // motor.setInverted(true);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void monitor() {
        SmartDashboard.putNumber("Shooter/Hood/Position", this.getMeasurement() + 45);
        SmartDashboard.putNumber("Shooter/Hood/RawPosition", this.getMeasurement());
        SmartDashboard.putNumber("Shooter/Hood/Error", getController().getPositionError());
        SmartDashboard.putNumber("Shooter/Hood/Setpoint", getController().getSetpoint());
        SmartDashboard.putNumber("Shooter/Hood/PWM/MotorPWMBoundMax", motor.getRawBounds().max);
        SmartDashboard.putNumber("Shooter/Hood/PWM/MotorPWMBoundDeadMax", motor.getRawBounds().deadbandMax);
        SmartDashboard.putNumber("Shooter/Hood/PWM/MotorPWMBoundCenter", motor.getRawBounds().center);
        SmartDashboard.putNumber("Shooter/Hood/PWM/MotorPWMBoundDeadMin", motor.getRawBounds().deadbandMin);
        SmartDashboard.putNumber("Shooter/Hood/PWM/MotorPWMBoundMin", motor.getRawBounds().min);
        SmartDashboard.putNumber("Shooter/Hood/PWM/MotorPWMSet", motor.getSpeed());
        SmartDashboard.putNumber("Shooter/Hood/PWM/OutputValue", lastOutput);
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
        // if (output > -0.05 && this.getMeasurement() >= MAX_GOAL) output = 0.0;
        // if (output < 0.05 && this.getMeasurement() <= MIN_GOAL) output = 0.0;
        // if (Utils.withinThreshold(this.getMeasurement(), setpoint, 0.5)) output = 0.0;
        lastOutput = output;
        motor.setSpeed(output);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getDistance();
    }
}
