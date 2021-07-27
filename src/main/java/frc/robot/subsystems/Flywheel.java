/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.util.Utils;

public class Flywheel extends TerribleSubsystem {
    private static final double SPEED_THRESHOLD = 50.0;
    private CANSparkMax shooterMaster, shooterFollower;
    private CANEncoder encoder;
    private static final double kP = 2.3 / 1000.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1 / 1000.0;
    private static final double kIz = 0.0;
    private static final double kFF = 1.08/5676.0;
    private static final double kMaxOutput = 1.0;
    private static final double kMinOutput = 0.0;
    private static final double GEAR_RATIO = 18.0 / 35.0;

    private double speedSetpoint = 0.0;

    public double x, y, area;

    public Flywheel() {
        setName("Shooter/Flywheel");
        shooterMaster = new CANSparkMax(1, MotorType.kBrushless);
        shooterMaster.restoreFactoryDefaults();
        shooterMaster.setInverted(false);
        shooterMaster.setIdleMode(IdleMode.kCoast);
        shooterMaster.setSmartCurrentLimit(40);
        shooterFollower = new CANSparkMax(2, MotorType.kBrushless);
        shooterFollower.restoreFactoryDefaults();
        shooterFollower.follow(shooterMaster, true); // Do not touch, this means inverted from master
        shooterFollower.setIdleMode(IdleMode.kCoast);
        shooterFollower.setSmartCurrentLimit(40);

        shooterMaster.getPIDController().setP(kP);
        shooterMaster.getPIDController().setI(kI);
        shooterMaster.getPIDController().setD(kD);
        shooterMaster.getPIDController().setIZone(kIz);
        shooterMaster.getPIDController().setFF(kFF);
        shooterMaster.getPIDController().setOutputRange(kMinOutput, kMaxOutput);
        shooterMaster.burnFlash(); // Save in case of brownout!
        shooterFollower.burnFlash();

        encoder = shooterMaster.getEncoder();
    }

    /**
     * Set the flywheels to speed control
     * @param RPM target rpm of flywheel
     */
    public void setSpeed(double RPM) {
        speedSetpoint = RPM;
        shooterMaster.getPIDController().setReference(speedSetpoint * GEAR_RATIO, ControlType.kVelocity);
    }

    Boolean isAtSpeed() {
        return Utils.withinThreshold(getVelocity(), speedSetpoint, SPEED_THRESHOLD);
    }

    public double getVelocity() {
        return encoder.getVelocity() / GEAR_RATIO;
    }

    @Override
    public void periodic() {
        display("Velocity", getVelocity());
        display("Setpoint", speedSetpoint);
        display("Output", shooterMaster.getAppliedOutput());
    }

    public void disable() {
        shooterMaster.disable();
    }
}
