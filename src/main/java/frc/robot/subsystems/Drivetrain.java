/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.controls.CubicSplineFollower;
import frc.robot.Constants;
import frc.util.Tuple;
import frc.util.Utils;

public class Drivetrain extends TerribleSubsystem {
	WPI_TalonFX rightMaster, rightFollower, leftMaster, leftFollower;
	CANifier canifierRight, canifierLeft;

	protected static final int UPDATE_RATE = 200;
	public DrivetrainModel model;
	private static final double ENCODER_TICKS_PER_REV = 8192.0;
	private Shifter shifter;

	public CubicSplineFollower waypointNav;

	// The time shouldn't matter, see
	// https://phoenix-documentation.readthedocs.io/en/latest/ch21_Errata.html#stator-current-limit-threshold-configs
	private StatorCurrentLimitConfiguration HIGH_GEAR_CURRENT_LIM = new StatorCurrentLimitConfiguration(
																			true, 50.0, 50.0, 0.001);
	private StatorCurrentLimitConfiguration LOW_GEAR_CURRENT_LIM = new StatorCurrentLimitConfiguration(
																			true, 35.0, 35.0, 0.001);

	private static final int PID_ID = 0; // Primary closed loop
	private static final int PID_SLOT_LOW = 0;
	private static final int PID_SLOT_HIGH = 1;
	private static final int CONFIG_TIMEOUT = 30;

	// TODO CHECK, should be (1023 * duty-cycle /  sensor-velocity-sensor-units-per-100ms).
	private static final double kFF_LOW = 1023.0 / linearSpeedToTalonSpeed(DrivetrainModel.MAX_SPEED_LOW);
	private static final double kP_LOW = 0.2;
	private static final double kD_LOW = 0.0;
	private static final double kI_LOW = 0.0;
	private static final double kFF_HIGH = 1023.0 / linearSpeedToTalonSpeed(DrivetrainModel.MAX_SPEED_HIGH);
	private static final double kP_HIGH = 0.01;
	private static final double kD_HIGH = 0.0;
	private static final double kI_HIGH = 0.0;

	private double time;

	private AHRS navx;

	private DriveControlState controlState = DriveControlState.DISABLED;

	private enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		PATH_FOLLOWING, // velocity PID control
		DISABLED,
	}

	public Drivetrain() {
		setName("Drivetrain");
		shifter = new Shifter();

		rightMaster = new WPI_TalonFX(Constants.Drivetrain.RIGHT_MOTOR_MASTER);
		rightFollower = new WPI_TalonFX(Constants.Drivetrain.RIGHT_MOTOR_FOLLOWER);
		leftMaster = new WPI_TalonFX(Constants.Drivetrain.LEFT_MOTOR_MASTER);
		leftFollower = new WPI_TalonFX(Constants.Drivetrain.LEFT_MOTOR_FOLLOWER);
		rightMaster.configFactoryDefault(CONFIG_TIMEOUT);
		rightFollower.configFactoryDefault(CONFIG_TIMEOUT);
		leftMaster.configFactoryDefault(CONFIG_TIMEOUT);
		leftFollower.configFactoryDefault(CONFIG_TIMEOUT);
		rightFollower.set(ControlMode.Follower, Constants.Drivetrain.RIGHT_MOTOR_MASTER);
		leftFollower.set(ControlMode.Follower, Constants.Drivetrain.LEFT_MOTOR_MASTER);
		rightMaster.setInverted(true);
		leftMaster.setInverted(false);
		rightFollower.setInverted(InvertType.FollowMaster);
		leftFollower.setInverted(InvertType.FollowMaster);
		setBrakeMode(false);

		canifierLeft = new CANifier(2);
		canifierRight = new CANifier(1);
		rightMaster.configRemoteFeedbackFilter(canifierRight.getDeviceID(),
											   RemoteSensorSource.CANifier_Quadrature, 0);
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_ID, 10);
		rightMaster.setSensorPhase(true);
		leftMaster.configRemoteFeedbackFilter(canifierLeft.getDeviceID(),
											   RemoteSensorSource.CANifier_Quadrature, 0);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_ID, 10);
		leftMaster.setSensorPhase(true);

		leftMaster.config_kF(PID_SLOT_LOW, kFF_LOW, CONFIG_TIMEOUT);
		leftMaster.config_kP(PID_SLOT_LOW, kP_LOW, CONFIG_TIMEOUT);
		leftMaster.config_kD(PID_SLOT_LOW, kD_LOW, CONFIG_TIMEOUT);
		leftMaster.config_kI(PID_SLOT_LOW, kI_LOW, CONFIG_TIMEOUT);
		rightMaster.config_kF(PID_SLOT_LOW, kFF_LOW, CONFIG_TIMEOUT);
		rightMaster.config_kP(PID_SLOT_LOW, kP_LOW, CONFIG_TIMEOUT);
		rightMaster.config_kD(PID_SLOT_LOW, kD_LOW, CONFIG_TIMEOUT);
		rightMaster.config_kI(PID_SLOT_LOW, kI_LOW, CONFIG_TIMEOUT);
		leftMaster.config_kF(PID_SLOT_HIGH, kFF_HIGH, CONFIG_TIMEOUT);
		leftMaster.config_kP(PID_SLOT_HIGH, kP_HIGH, CONFIG_TIMEOUT);
		leftMaster.config_kD(PID_SLOT_HIGH, kD_HIGH, CONFIG_TIMEOUT);
		leftMaster.config_kI(PID_SLOT_HIGH, kI_HIGH, CONFIG_TIMEOUT);
		rightMaster.config_kF(PID_SLOT_HIGH, kFF_HIGH, CONFIG_TIMEOUT);
		rightMaster.config_kP(PID_SLOT_HIGH, kP_HIGH, CONFIG_TIMEOUT);
		rightMaster.config_kD(PID_SLOT_HIGH, kD_HIGH, CONFIG_TIMEOUT);
		rightMaster.config_kI(PID_SLOT_HIGH, kI_HIGH, CONFIG_TIMEOUT);
		model = new DrivetrainModel();
		model.setPosition(0.0, 0.0, 0.0);

		waypointNav = new CubicSplineFollower(model);

		navx = new AHRS(SPI.Port.kMXP);

		this.updateThreadStart();

		setShiftMode(true);
		disable();
	}

	@Override
	protected void update() {
		double time = Timer.getFPGATimestamp();
		double deltaTime = time - this.time;
		this.time = time;
		this.updateOdometry(deltaTime);
		display("DT", deltaTime);

		switch (controlState) {
			case OPEN_LOOP:
				break;
			case PATH_FOLLOWING:
				driveWaypointNavigator();
				break;
			case DISABLED:
				break;
		}
	}

	private void updateOdometry(double time) {
		double leftSpeed = talonSpeedToLinearSpeed(leftMaster.getSelectedSensorVelocity());
		double rightSpeed = talonSpeedToLinearSpeed(rightMaster.getSelectedSensorVelocity());
		model.updateSpeed(leftSpeed, rightSpeed, time);
		model.updateHeading(navx.getAngle());
		model.updatePosition(time);
	}

	public void zeroSensors() {
		leftMaster.setSelectedSensorPosition(0);
		rightMaster.setSelectedSensorPosition(0);
		navx.reset();
	}

	public void disable() {
		controlState = DriveControlState.DISABLED;
		leftMaster.stopMotor();
		leftFollower.stopMotor();
		rightMaster.stopMotor();
		rightFollower.stopMotor();
	}

	private boolean testSide(WPI_TalonFX talon, String name, double speed) {
		Faults faults = new Faults();
		/* update motor controller */
		talon.set(ControlMode.PercentOutput, speed);
		boolean error = false;
		try {
			Thread.sleep(1000);
			ErrorCode errors = talon.getFaults(faults);
			error = errors.value != 0;
			error &= Utils.withinThreshold(talonSpeedToLinearSpeed(talon.getSelectedSensorVelocity()),
					model.topSpeed * speed, model.topSpeed * speed * 0.05);
		} catch (InterruptedException e) {
			error = true;
		}
		// System.out.println(name + "Sensor Vel:" + talon.getSelectedSensorVelocity());
		// System.out.println(name + "Sensor Pos:" + talon.getSelectedSensorPosition());
		// System.out.println(name + "Out %" + talon.getMotorOutputPercent());
		// System.out.println(name + "Out Of Phase:" + faults.SensorOutOfPhase);
		talon.set(ControlMode.PercentOutput, 0.0);
		if (error) DriverStation.reportError(getName() + "Test " + name, true);
		return error;
	}

	protected boolean test() {
		setShiftMode(true);
		testSide(rightMaster, "Right Forward", 1.0);
		testSide(rightMaster, "Right Reverse", -1.0);
		testSide(leftMaster, "Left Forward", 1.0);
		testSide(leftMaster, "Left Reverse", -1.0);
		return true;
	}

	public void setBrakeMode(Boolean enabled) {
		// BANANA TODO is half brake useful??
		NeutralMode mode;
		if (enabled)
			mode = NeutralMode.Brake;
		else
			mode = NeutralMode.Coast;
		rightMaster.setNeutralMode(mode);
		rightFollower.setNeutralMode(mode);
		leftMaster.setNeutralMode(mode);
		leftFollower.setNeutralMode(mode);
	}

	public void startPathFollowing() {
		if (controlState != DriveControlState.PATH_FOLLOWING) {
			System.out.println("Switching to path following, time: " + time);
			controlState = DriveControlState.PATH_FOLLOWING;
		}
		// BANANA TODO print waypoints
	}

	private void driveWaypointNavigator() {
		Tuple output = waypointNav.updatePursuit(model.center);
		double leftSpeed = output.left;
		double rightSpeed = output.right;

		display("Left Out 1", leftSpeed);
		display("Right Out 1", rightSpeed);
		setSpeed(leftSpeed, rightSpeed);
	}

	private double m_quickStopThreshold = 0.2;
	private double m_quickStopAlpha = 0.1;
	private double m_quickStopAccumulator;

	public void terribleDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		double m_deadband = 0.02;
		xSpeed = Utils.limit(xSpeed);
		xSpeed = Utils.applyDeadband(xSpeed, m_deadband);

		zRotation = Utils.limit(zRotation);
		zRotation = Utils.applyDeadband(zRotation, m_deadband);

		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(xSpeed) < m_quickStopThreshold) {
				m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
						+ m_quickStopAlpha * zRotation * 2;
			}
			overPower = true;
			angularPower = zRotation;
		} else {
			overPower = false;
			angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1) {
				m_quickStopAccumulator -= 1;
			} else if (m_quickStopAccumulator < -1) {
				m_quickStopAccumulator += 1;
			} else {
				m_quickStopAccumulator = 0.0;
			}
		}

		double leftOutput = xSpeed + angularPower;
		double rightOutput = xSpeed - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftOutput > 1.0) {
				rightOutput -= leftOutput - 1.0;
				leftOutput = 1.0;
			} else if (rightOutput > 1.0) {
				leftOutput -= rightOutput - 1.0;
				rightOutput = 1.0;
			} else if (leftOutput < -1.0) {
				rightOutput -= leftOutput + 1.0;
				leftOutput = -1.0;
			} else if (rightOutput < -1.0) {
				leftOutput -= rightOutput + 1.0;
				rightOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
		if (maxMagnitude > 1.0) {
			leftOutput /= maxMagnitude;
			rightOutput /= maxMagnitude;
		}

		setOpenLoop(leftOutput, rightOutput);
	}

	private void setSpeed(double left, double right) {
		double leftTalonSpeed = linearSpeedToTalonSpeed(left);
		double rightTalonSpeed = linearSpeedToTalonSpeed(right);
		leftMaster.set(ControlMode.Velocity, leftTalonSpeed);
		rightMaster.set(ControlMode.Velocity, rightTalonSpeed);
	}

	private static double linearSpeedToTalonSpeed(double linearSpeed) {
		double wheelRotationalSpeed = linearSpeed / DrivetrainModel.WHEEL_CIRCUMFERENCE;
		double encoderRotationSpeed = wheelRotationalSpeed * ENCODER_TICKS_PER_REV;
		double talonSpeed = encoderRotationSpeed / 10.0;
		return talonSpeed;
	}

	private static double talonSpeedToLinearSpeed(double talonSpeed) {
		double ticksPerSecond = talonSpeed * 10.0;
		double wheelRotationalSpeed = ticksPerSecond / ENCODER_TICKS_PER_REV;
		double linearSpeed = wheelRotationalSpeed * DrivetrainModel.WHEEL_CIRCUMFERENCE;
		return linearSpeed;
	}

	private void setOpenLoop(double left, double right) {
		if (controlState != DriveControlState.OPEN_LOOP) {
			System.out.println("Switching to open loop control, time: " + time);
			controlState = DriveControlState.OPEN_LOOP;
		}
		leftMaster.set(ControlMode.PercentOutput, left);
		rightMaster.set(ControlMode.PercentOutput, right);
	}

	/**
	 * @param true if high gear (faster dt) desired
	 */
	public void setShiftMode(boolean high) {
		model.shiftMode(high);
		shifter.setGear(high);
		int pidSlot = PID_SLOT_LOW;
		StatorCurrentLimitConfiguration curLim = LOW_GEAR_CURRENT_LIM;
		if (high) {
			pidSlot = PID_SLOT_HIGH;
			curLim = HIGH_GEAR_CURRENT_LIM;
		}
		leftMaster.selectProfileSlot(pidSlot, PID_ID); // TODO Check if this works
		leftMaster.configStatorCurrentLimit(curLim, CONFIG_TIMEOUT);
		rightMaster.configStatorCurrentLimit(curLim, CONFIG_TIMEOUT);
	}

	@Override
	public void periodic() {
		super.periodic();
		display("Left Encoder", talonSpeedToLinearSpeed(leftMaster.getSelectedSensorVelocity()));
		display("Rights Encoder", talonSpeedToLinearSpeed(rightMaster.getSelectedSensorVelocity()));
		display("Model X", model.center.x);
		display("Model Y", model.center.y);
		display("Heading", model.center.heading);
		display("Radians", model.center.r);
		display("Left Output", leftMaster.getStatorCurrent());
		display("Right Output", rightMaster.getStatorCurrent());
	}

	public static void main(String[] args) throws IOException {

	}
}