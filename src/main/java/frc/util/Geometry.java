package frc.util;

public class Geometry {

	/**
	 * Returns side b, calculated using law of sines
	 *
	 * @param a    length of side a
	 * @param sinA value for sine of angle A, opposite of side a
	 * @param sinB value for sine of angle B, opposite of side b
	 * @return the length of side b
	 */
	public static double sideFromLawOfSines(double a, double sinA, double sinB) {
		return (a * sinB / sinA);
	}

	public static double distance(Pose a, Pose b) {
		return distance(a.x, b.x, a.y, b.y);
	}

	public static double distance(double x1, double x2, double y1, double y2) {
		double deltaX = x1 - x2;
		double deltaY = y1 - y2;
		double dist = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
		return dist;
	}

	public static double hypotenuse(double a, double b) {
		return distance(0.0, a, 0.0, b);
	}


	/**
	 * Takes an angle and converts it to base degrees (-180 - 180)
	 *
	 * @param angle angle in degrees
	 * @return limited angle in degrees
	 */
	public static double limitAngleDeg(double angle) {
		return limitAngle(angle, -180.0, 180.0, 360.0);
	}

	/**
	 * Takes an angle and converts it to base degrees (-180 - 180)
	 *
	 * @param angle angle in degrees
	 * @return limited angle in degrees
	 */
	public static double limitAngleRad(double angle) {
		return limitAngle(angle, -Math.PI, Math.PI, 2.0*Math.PI);
	}

	public static double angleBetweenDegrees(Pose looker, Pose reference) {
		return Math.toDegrees(Math.atan2(reference.x - looker.x, reference.y - looker.y));
	}

	public static double limitAngle(double angle, double min, double max, double revUnit) {
		angle = angle % revUnit;
		if (angle < min) angle += revUnit;
		else if (angle > max) angle -= revUnit;
		return angle;
	}

	// Testing calculations
	public static void main(String[] args) {
		System.out.println(limitAngle(271, -90, 270, 360));
	}
}