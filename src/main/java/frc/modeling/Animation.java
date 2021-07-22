package frc.modeling;

import javax.imageio.ImageIO;
import javax.swing.*;

import frc.controls.CubicSplineFollower;
import frc.controls.CubicSplineFollower.Waypoint;
import frc.robot.subsystems.DrivetrainModel;
import frc.util.Tuple;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class Animation extends JPanel implements Runnable {

	/**
	 *
	 */
	private static final long serialVersionUID = 1L;

	public static void main(String[] args) throws IOException {
		JFrame frame = new JFrame("Drivetrain Simulation");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		Container pane = frame.getContentPane();
		Animation panel = new Animation();
		pane.add(panel, BorderLayout.CENTER);

		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}

	private Thread sim; // animation thread

	private static final String FIELD_FILE = "sim/2020-field.png";
	private static final double FIELD_REAL_WIDTH = 8.23;
	// private static final double FIELD_REAL_HEIGHT = 13.0;
	private int fieldImageWidth, fieldImageHeight; // image dim of viewing area in pixels
	private Dimension screenSize; // size of viewing area
	private Image image; // off-screen image
	private Graphics offScreen; // off-screen graphics
	private Image field;
	private BufferedImage robot;
	private final double scale; // pixels per meter
	private int robotImageWidth, robotImageHeight;

	private ArrayList<Tuple> trajectory;// array for storing points passed through by robot
	private final int SPEED = 1; // replay speed
	private final int FRAME_RATE = 60; // interval between frames in millisec

	private DrivetrainModel model;
	private CubicSplineFollower nav;

	private static final int ODOMETRY_UPDATE_RATE = 200;
	private static final int CONTROL_UPDATE_RATE = 200;
	private double leftVoltage = 0.0;
	private double rightVoltage = 0.0;

	private double time = 0.0;

	public Animation() throws IOException {
		field = ImageIO.read(new File(FIELD_FILE));
		robot = ImageIO.read(new File("sim/robot-blue2.png"));

		// Set the width and heigth and size
		fieldImageWidth = field.getWidth(this);
		fieldImageHeight = field.getHeight(this);
		robotImageWidth = robot.getWidth(this);
		robotImageHeight = robot.getHeight(this);
		scale = fieldImageWidth / FIELD_REAL_WIDTH;

		model = new DrivetrainModel();
		nav = new CubicSplineFollower(model);

		this.setWaypoints();

		sim = new Thread(this); // Create and start the thread
		sim.start();
		odometryThread();
		controlThread();

		trajectory = new ArrayList<Tuple>();
	}

	private void setWaypoints() {
		model.setPosition(FieldPositions.RIGHT.START);
		for (Waypoint waypoint : FieldPositions.RIGHT.WAYPOINTS) nav.addWaypoint(waypoint);
	}


	// Update function
	public void paintComponent(Graphics g) {
		super.paintComponent(g);

		screenSize = this.getSize(); // Get the size of the viewing area
		if (image == null) { // Create the off-screen image buffer if it is the first time
			image = createImage(fieldImageWidth, fieldImageHeight);
			offScreen = image.getGraphics();
		}

		offScreen.drawImage(field, 0, 0, this); // Draw background field

		/// Draw robot
		BufferedImage robotRotated = rotateRobot();
		Tuple robotLoc = fieldCoordsToImageCoords(model.center.x, model.center.y);
		int robotImageOffsetX = (int) robotLoc.left - (robotRotated.getWidth() / 2);
		int robotImageOffsetY = (int) robotLoc.right - (robotRotated.getHeight() / 2);
		if (!nav.isFinished)
			trajectory.add(new Tuple(robotLoc.left, robotLoc.right));
		offScreen.drawImage(robotRotated, robotImageOffsetX, robotImageOffsetY, this);

		// Draw waypoints
		Waypoint curWaypoint = nav.getCurrentWaypoint();
		if (curWaypoint != null) {
			Tuple waypointLoc = fieldCoordsToImageCoords(curWaypoint.x, curWaypoint.y);
			offScreen.setColor(Color.yellow);
			offScreen.drawOval((int) waypointLoc.left - 3, (int) waypointLoc.right - 3, 6, 6);
		}

		// Draw trajectory
		for (Tuple point : trajectory) {
			offScreen.setColor(Color.yellow);
			offScreen.drawOval((int) point.left, (int) point.right, 1, 1);
		}

		// Copy the off-screen image to the screen and scale to fit jframe
		Graphics2D g2 = (Graphics2D) g;
		g2.scale(screenSize.width / (double) this.fieldImageWidth, screenSize.height / (double) this.fieldImageHeight);
		g2.drawImage(image, 0, 0, this);
	}

	private Tuple fieldCoordsToImageCoords(double x, double y) {
		int newX = (int) ((fieldImageWidth / 2) + x * scale);
		int newY = (int) (fieldImageHeight * 0.0 - (y - 0.73) * scale);
		return new Tuple(newX, newY);
	}

	public BufferedImage rotateRobot() {
		double sin = Math.abs(Math.sin(model.center.r));
		double cos = Math.abs(Math.cos(model.center.r));

		int newWidth = (int) Math.floor(robotImageWidth * cos + robotImageHeight * sin);
		int newHeight = (int) Math.floor(robotImageHeight * cos + robotImageWidth * sin);

		BufferedImage rotated = new BufferedImage(newWidth, newHeight, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2d = rotated.createGraphics();
		AffineTransform at = new AffineTransform();
		double robotScale = scale / robotImageHeight * 1.0;
		at.translate((newWidth - robotImageWidth * robotScale) / 2, (newHeight - robotImageHeight * robotScale) / 2);
		at.scale(robotScale, robotScale);

		int x = robotImageWidth / 2;
		int y = robotImageHeight / 2;

		at.rotate(model.center.r, x, y);
		g2d.setTransform(at);
		g2d.drawImage(robot, 0, 0, this);
		g2d.dispose();

		return rotated;
	}

	@Override
	public Dimension getPreferredSize() {
		return new Dimension(fieldImageWidth, fieldImageHeight);
	}

	@Override
	public Dimension getMinimumSize() {
		return new Dimension(fieldImageWidth / 3, fieldImageHeight / 3);
	}

	@Override
	public Dimension getMaximumSize() {
		return new Dimension(fieldImageWidth, fieldImageHeight);
	}

	@Override
	public void run() {
		while (Thread.currentThread() == sim && nav.isFinished == false) {
			repaint();
			try {
				Thread.sleep(1000 / FRAME_RATE);
			} catch (InterruptedException e) {
				System.out.println("Exception: " + e.getMessage());
			}
		}
	}

	public void controlThread() {
		Thread c = new Thread(() -> {
			while (!Thread.interrupted()) {

				Tuple output = nav.updatePursuit(model.center);
				model.updateSpeed(output.left, output.right, 1.0/CONTROL_UPDATE_RATE);

				if (nav.isFinished) {
				}

				try {
					Thread.sleep(1000 / CONTROL_UPDATE_RATE * SPEED);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});
		c.start();
	}

	public void odometryThread() {
		Thread o = new Thread(() -> {
			while (!Thread.interrupted()) {
				time += 1000 / ODOMETRY_UPDATE_RATE;


				// model.updateVoltage(leftVoltage, rightVoltage, 1.0 / ODOMETRY_UPDATE_RATE);
				model.updatePosition(1.0 / ODOMETRY_UPDATE_RATE);
				// System.out.println(model.center.x);

				try {
					Thread.sleep(1000 / ODOMETRY_UPDATE_RATE * SPEED);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});
		o.start();
	}
}