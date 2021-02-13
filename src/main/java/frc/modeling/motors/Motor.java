package frc.modeling.motors;

/**
 * Abstract class for motor implementation, with methods for calculating torque
 * and in the future tracking thermal charecteristics.
 * To implement, update constants in declarator and run calculateSlopes()
 * 
 * @author eric
 *
 */
abstract public class Motor {
	protected static double STALL_TORQUE; // N. m
	protected static double FREE_SPEED; // RPM
	protected static double STALL_CURRENT; // Amps
	protected static double FREE_CURRENT; // Amps
	private static final double MAX_VOLTAGE = 12.0; // Volts
	protected static double kSlopeTorque; // -STALL_TORQUE / FREE_SPEED;
	protected static double kSlopeCurrent; // -(STALL_CURRENT - FREE_CURRENT) / FREE_SPEED;

	public static double outputTorque (double voltage, double speed) {
		double stallTorque = STALL_TORQUE * (voltage / MAX_VOLTAGE);
		double torque = (speed * kSlopeTorque) + stallTorque;
		return torque;
	}

	public static double torqueToVoltage (double torque, double speed) {
		return (torque - (speed*kSlopeTorque))*MAX_VOLTAGE/STALL_TORQUE;
		// Just solved for voltage using outputTorque equation
	}

	public static double currentToVoltage (double current, double speed) {
		return (current - (speed*kSlopeCurrent))*MAX_VOLTAGE/STALL_CURRENT;
	}
}