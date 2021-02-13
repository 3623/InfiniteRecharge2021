package frc.util;

import java.io.IOException;

/**
 * Simple Cartesian coordinate class
 *
 * @author eric
 *
 */
public class Pose {
	public double x;
	public double y;
	public double r;
	public double heading;
	public double velocity;
	public double angularVelocity;

	/**
	 * Constructor for CartesianCoordinate class
	 * @param x the x location in meters
	 * @param y the y location in meters
	 * @param heading the heading angle in degress, is converted into radians
	 */
	public Pose(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.r = Math.toRadians(heading);
		this.heading = heading;
		limitAngle();
	}

	// public Pose(double x, double y, double r){
	// 	this.x = x;
	// 	this.y = y;
	// 	this.r = r;
	// 	this.heading = Math.toDegrees(r);
	// }

	public Pose(double x, double y) {
		this(x, y, 0.0);
	}

	public Pose() {
		this(0.0, 0.0, 0.0);
	}

	public void update(double deltaX, double deltaY, double deltaR) {
		this.x += deltaX;
		this.y += deltaY;
		this.r += deltaR;
		this.heading = Math.toDegrees(r);
		limitAngle();
	}

	public void setHeading(double heading) {
		this.heading = heading;
		this.r = Math.toRadians(this.heading);
		limitAngle();
	}

	private void limitAngle() {
		this.heading = Geometry.limitAngleDegrees(this.heading);
		this.r = Math.toRadians(this.heading);
	}

	public String toString() {
		return "x: " + x + ", y: " + y + ", heading: " + heading + ", radians: " + r;
	}

	public static void main(String[] args) throws IOException {
		System.out.println();
	}
}