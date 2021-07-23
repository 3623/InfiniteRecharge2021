package frc.robot.subsystems;

import frc.modeling.motors.Falcon;
import frc.util.Geometry;
import frc.util.Pose;
import frc.util.Utils;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Model for a differential drivetrain, simulates mass and motor performance
 * Integrates acceleration over time for both sides of the drivetrain and uses
 * velocity to calculate the instantaneous curvature of the robot, which is then
 * integrated for position
 *
 * @author eric
 *
 */
public class DrivetrainModel {

	private static final double DRIVETRAIN_MASS = 63.5; // kg
	protected static final double MAX_SPEED_LOW = 2.4;
	protected static final double MAX_SPEED_HIGH = 4.0;

	public static final double WHEEL_BASE = 0.65; // meters
	public static final double WHEEL_RADIUS = 0.081; // meters
	public static final double WHEEL_CIRCUMFERENCE = DrivetrainModel.WHEEL_RADIUS * 2.0 * Math.PI;

	private Boolean COAST_MODE = false;

	private static final double MOTORS_PER_SIDE = 2.0; // Minicim is 0.58
	private static final double BASE_GEAR_RATIO = 10.75 / 1.0; // Reduction
	private static final double HIGH_GEAR_RATIO_MODIFIER = 2.0;
	private static final double DRIVETRAIN_FRICTION = 115;

	private double topSpeed;
	private boolean gear;
	private double gearRatio;
	private DrivetrainSide left, right;
	public Pose center;

	public DrivetrainModel() {
		center = new Pose(0.0, 0.0, 0.0); // Initial robot position
		left = new DrivetrainSide();
		right = new DrivetrainSide();

		if (COAST_MODE) {
			left.coast = true;
			right.coast = true;
		} else {
			left.coast = false;
			right.coast = false;
		}
		shiftMode(true);
	}

	public void shiftMode(boolean high) {
		if (high) {
			topSpeed = MAX_SPEED_HIGH;
			gearRatio = BASE_GEAR_RATIO / HIGH_GEAR_RATIO_MODIFIER;
			gear = true;
		} else {
			topSpeed = MAX_SPEED_LOW;
			gearRatio = BASE_GEAR_RATIO;
			gear = false;
		}
	}

	/**
	 * Sets the drivetrain position to a known location
	 *
	 * @param x        location (side to side), in meters
	 * @param y        location (forwards backwards, in meters
	 * @param heading, in degrees
	 */
	public void setPosition(double x, double y, double heading) {
		center.x = x;
		center.y = y;
		center.setHeading(heading);
		zero();
	}

	public void setPosition(Pose pose) {
		setPosition(pose.x, pose.y, pose.heading);
	}

	/**
	 * Zeros the velocity and acceleration of each drivetrain side
	 */
	public void zero() {
		left.velocity = 0.0;
		right.velocity = 0.0;
		left.acceleration = 0.0;
		right.acceleration = 0.0;
	}

	/**
	 * Sets the speed of each drivetrain side to a known value and calculates
	 * acceleration. For using encoders
	 *
	 * @param lSpeed left speed, in meters/sec
	 * @param rSpeed right speed, in meters/sec
	 * @param time   elapsed time since last update, in seconds
	 */
	public void updateSpeed(double lSpeed, double rSpeed, double time) {
		left.updateSpeed(lSpeed, time);
		right.updateSpeed(rSpeed, time);
	}

	/**
	 * Updates the models heading with a set heading. For using a gyro to more
	 * accurately track heading
	 *
	 * @param heading angle in degrees
	 */
	public void updateHeading(double heading) {
		center.setHeading(heading);
	}

	/**
	 * Updates velocity and acceleration of each side of the drivetrain using motor
	 * curves
	 *
	 * @param lVoltage left voltage
	 * @param rVoltage right voltage
	 * @param time     elapsed time between updates, in seconds
	 */
	public void updateVoltage(double lVoltage, double rVoltage, double time) {
		left.updateVoltage(lVoltage, time);
		right.updateVoltage(rVoltage, time);
	}

	/**
	 * Updates the models position from each sides velocity
	 *
	 * @param time elapsed time between updates, in seconds
	 */
	public void updatePosition(double time) {
		double radius = radiusICC(WHEEL_BASE, left.velocity, right.velocity);
		double omega = velocityICC(WHEEL_BASE, left.velocity, right.velocity);
		double theta = omega * (time);
		double sinTheta = Math.sin(theta);
		double alpha = ((Math.PI) - theta) / 2.0;
		double sinAlpha = Math.sin(alpha);

		double movementAngle = center.r + theta;
		double movement = Geometry.sideFromLawOfSines(radius, sinAlpha, sinTheta);

		if (omega == 0.0) {
			movement = -(left.velocity + right.velocity) / 2.0 * time;
		}
		double sine = Math.sin(movementAngle);
		double cosine = Math.cos(movementAngle);
		double movementX = -movement * sine;
		double movementY = -movement * cosine;
		center.update(movementX, movementY, -theta);

		center.velocity = (left.velocity + right.velocity) / 2.0;
		center.angularVelocity = -Math.toDegrees(theta) / time;

		// // Debug statements
		// System.out.println("X: " + center.x + ", Y: " + center.y + ", Heading: " +
		// center.heading);
		// System.out.println("MA: " + movementAngle + ", Rad ICC: " + radius);
		// System.out.println("Movement X: " + movementX + ", Movement Y: " +
		// movementY);
	}

	public double getTopSpeed() {
		return topSpeed;
	}

	public boolean getGear(){
		return gear;
	}


	private class DrivetrainSide {
		double velocity;
		double acceleration;
		private double psuedoMass;
		private Boolean coast;

		public DrivetrainSide() {
			velocity = 0.0;
			acceleration = 0.0;
			psuedoMass = DRIVETRAIN_MASS / 2;
			coast = false;
			new Falcon();
		}

		/**
		 * Sets the speed of each drivetrain side to a known value and calculates
		 * acceleration. For using encoders
		 *
		 * @param speed, in meters/sec
		 */
		protected void updateSpeed(double speed, double time) {
			double deltaVelocity = this.velocity - speed;
			this.acceleration = deltaVelocity / time;
			this.velocity = speed;
		}

		/**
		 * Updates velocity and acceleration of each side of the drivetrain using motor
		 * curves
		 *
		 * @param voltage
		 * @param the     elapsedtime between updates, in seconds
		 */
		protected void updateVoltage(double voltage, double time) {
			voltage = Utils.limit(voltage, 14.0, -14.0);
			double motorSpeed = this.wheelSpeedToMotorSpeed(this.velocity);
			double totalTorque = Falcon.outputTorque(voltage, motorSpeed) * gearRatio * MOTORS_PER_SIDE;
			if (coast && Utils.withinThreshold(voltage, 0.0, 0.05))
				totalTorque = 0.0;
			double wheelForce = (totalTorque / WHEEL_RADIUS);

			double wheelnetForce = frictionModel(wheelForce, this.velocity);

			double newAcceleration = wheelnetForce / psuedoMass;
			this.velocity += (newAcceleration + this.acceleration) / 2 * time; // Trapezoidal integration
			this.acceleration = newAcceleration;
		}

		/**
		 * Converts linear wheel speed back to motor angular speed
		 *
		 * @param speed meters/sec
		 * @return angular speed, revolutions per minute
		 */
		private double wheelSpeedToMotorSpeed(double speed) {
			double wheelCircum = WHEEL_RADIUS * 2 * Math.PI;
			double wheelRevs = speed / wheelCircum * 60.0;
			double motorRevs = wheelRevs * gearRatio;
			return motorRevs;
		}

		/**
		 * Models friction as a constant
		 *
		 * @param Ideal output force of the drivetrain, in newtons
		 * @param Speed of the drivetrain, to set direction of friction
		 */
		private double frictionModel(double force, double speed) {
			double netForce;
			if (speed == 0.0) {
				netForce = force;
			} else if (speed < 0.0) {
				netForce = force + DRIVETRAIN_FRICTION;
			} else if (speed > 0.0) {
				netForce = force - DRIVETRAIN_FRICTION;
			} else {
				netForce = 0.0;
			}
			return netForce;
		}
	}

	/**
	 * The radius of the robot about the Instantaneous Center of Curvature (ICC)
	 * Used to infinitesimally calculate the displacement of the robot A positive
	 * radius is to the right of the robot (relative to robot) and negative is left
	 *
	 * @see <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">
	 *      Columbia University: CS W4733 NOTES - Differential Drive Robots</a>
	 * @param wheelBase width between left and right sides of the drivetrain, meters
	 * @param left      velocity of the left wheel, m/s
	 * @param right     velocity of the right wheel, m/s
	 * @return the radius from the center of the robot to the ICC
	 */
	private static double radiusICC(double wheelBase, double left, double right) {
		return -(wheelBase / 2) * (left + right) / (right - left);
	}

	/**
	 * The angular velocity of the robot about the Instantaneous Center of Curvature
	 * (ICC) Used to infinitesimally calculate the displacement of the robot A
	 * positive radius is to the left of the robot (relative to robot) and negative
	 * is right
	 *
	 * @see <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">
	 *      Columbia University: CS W4733 NOTES - Differential Drive Robots</a>
	 * @param wheelBase width between left and right sides of the drivetrain, meters
	 * @param left      velocity of the left wheel, m/s
	 * @param right     velocity of the right wheel, m/s
	 * @return
	 */
	private static double velocityICC(double wheelBase, double left, double right) {
		return (right - left) / wheelBase;
	}

	// For Testing
	public static void main(String[] args) {
		DrivetrainModel model = new DrivetrainModel();
		for (int i = 0; i < 1; i++) {
			model.updateSpeed(3.0, 3.0, 0.02);
		}
	}
}
