package frc.modeling;

import frc.controls.CubicSplineFollower.Waypoint;
import frc.util.Pose;

public class FieldPositions {

	public interface PATH_SET{
		Pose START = null;
		Waypoint[] WAYPOINTS = null;
	}

	public static class LEFT implements PATH_SET{
		public static final Pose START = new Pose(-3.3, -3.5, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(-3.3, -5.8, 0.0, -0.8, true),
													new Waypoint(0.5, -3.0, 50.0, 0.8, true),
													new Waypoint(-0.2, -5.5, -25.0, -0.8, true),
													new Waypoint(1.0, -3.0, 0.0, 0.8, true)};
	}

	public static class MIDDLE implements PATH_SET{
		public static final Pose START = new Pose(2.5, -3.5, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(1.7, -5.9, 65.0, -0.8, true),
													new Waypoint(0.1, -5.9, 135.0, -0.8, false),
													new Waypoint(1.0, -3.0, -140.0, -0.8, true)};
	}

    // Middle

	public static class RIGHT implements PATH_SET {
		public static final Pose START = new Pose(3.4, -3.5, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(3.4, -6.5, 0.0, -1.0, true),
													new Waypoint(3.4, -5.0, 0.0, 1.0, true),
													new Waypoint(3.4, -9.1, 0.0, -1.0, true),
													new Waypoint(2.7, -5.0, 20.0, 1.0, true),
													new Waypoint(1.6, -5.9, 70.0, -1.0, true)};
	}

	public static class BLOCK implements PATH_SET {
		public static final Pose START = new Pose(3.4, -3.5, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(3.4, -10.5, 0.0, -1.0, true),
													new Waypoint(2.9, -5.0, 10.0, 1.0, true),
													new Waypoint(3.3, -8.2, -55.0, -1.0, true),
													new Waypoint(1.2, -5.9, -15.0, 1.0, true),
													new Waypoint(1.2, -5.9, -30.0, -1.0, true)};
	}

	public static class STEAL implements PATH_SET {
		public static final Pose START = new Pose(2.5, -3.0, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(1.6, -5.9, 70.0, -1.0, true),
													new Waypoint(-1.6, -8.8, -20.0, -1.0, false),
													new Waypoint(-0.6, -9.6, -70.0, -1.0, false),
													new Waypoint(1.0, -9.5, -110.0, -1.0, false),
													new Waypoint(2.0, -4.5, -180.0, -1.0, true)};
	}

	public static class BARREL implements PATH_SET {
		private static double speedResponse = 1;
		public static final Pose START = new Pose(0.0, -1.0, -10.0);
		public static final Waypoint[] WAYPOINTS = { new Waypoint(-0.9, -4.6, 90.0, -1.0 * speedResponse, false), // 1st
													//new Waypoint(-1.6, -3.5, -180.0, -1.0 * speedResponse, false),
													new Waypoint(-1.0, -2.8, -100.0, -0.7 * speedResponse, false),
													//new Waypoint (0.3, -3.5, 10.0, -1 * speedResponse, false),
													new Waypoint (0.0, -4.5, 0.0, -1.0 * speedResponse, false),
													new Waypoint(1.1, -6.85, -90.0, -0.7 * speedResponse, false), // 2nd
													//new Waypoint(1.7, -6.0, 180.0, -1.0 * speedResponse, false),
													new Waypoint(1.3, -5.2, 90.0, -1.0 * speedResponse, false),
													new Waypoint(-1.5, -8.1, 0.0, -1.0 * speedResponse, false), // 3rd
													//new Waypoint(-1.0, -8.3, -90.0, -1.0 * speedResponse, false),
													new Waypoint(0.0, -8.0, 180.0, -1.0 * speedResponse, false),
													new Waypoint(0.3, 0.5, -170.0, -1.0 * speedResponse, false) };

													/*working rough turns  new Waypoint(-0.85, -4.35, 90.0, -1.0 * speedResponse, false), // 1st
													new Waypoint(-1.6, -3.5, -180.0, -1.0 * speedResponse, false),
													new Waypoint(-1.0, -2.8, -100.0, -1.0 * speedResponse, false),
													new Waypoint (0.3, -3.5, 10.0, -1 * speedResponse, false),
													new Waypoint (0.0, -5.5, 0.0, -1 * speedResponse, false),
													new Waypoint(0.8, -6.7, -90.0, -1.0 * speedResponse, false), // 2nd
													new Waypoint(1.7, -6.0, 180.0, -1.0 * speedResponse, false),
													new Waypoint(1.0, -5.2, 90.0, -1.0 * speedResponse, false),
													new Waypoint(-1.5, -7.6, 0.0, -1.0 * speedResponse, false), // 3rd
													new Waypoint(-1.0, -8.3, -90.0, -1.0 * speedResponse, false),
													new Waypoint(-0.0, -7.6, 180.0, -1.0 * speedResponse, false),
													new Waypoint(0.0, 0.0, -170.0, -1.0 * speedResponse, false) };*/
	}

	public static class SLALOM implements PATH_SET {
		private static double speedResponse = 0.925;
		public static final Pose START = new Pose(-1.0, -1.0, -25.0);
		public static final Waypoint[] WAYPOINTS = { new Waypoint(0.75, -3.0, -30.0, -1.0 * speedResponse, false), // 1st
													new Waypoint(0.0, -6.8, 45.0, -1.0 * speedResponse, false),
													new Waypoint(-0.65, -8.0, -40.0, -0.6 * speedResponse, false),
													//new Waypoint(0.0, -8.5, -90.0, -1.0 * speedResponse, false),
													new Waypoint(0.65, -8.0, 165.0, -0.6 * speedResponse, false),
													new Waypoint(-0.6, -6.4, 145.0, -1.0 * speedResponse, false),
													new Waypoint(0.0, -1.9, -115.0, -1.0 * speedResponse, false),
													new Waypoint(0.9, 0.0, 180.0, -1.0 * speedResponse, false),};
		
		
		/* working but rough around the curve{ new Waypoint(0.75, -3.0, -30.0, -1.0, false), // 1st
													new Waypoint(0.0, -6.8, 45.0, -1.0, false),
													new Waypoint(-1.0, -7.5, 0.0, -1.0, false),
													new Waypoint(0.0, -8.5, -90.0, -1.0, false),
													new Waypoint(1.0, -7.5, 180.0, -1.0, false),
													new Waypoint(-0.5, -6.4, 150.0, -1.0, false),
													new Waypoint(0.0, -1.85, -115.0, -1.0, false),
													new Waypoint(0.75, 0.0, 180.0, -1.0, false),};*/
	}

	public static class BOUNCE implements PATH_SET {
		private static double vinceHasANeed = 1;
		public static final Pose START = new Pose(0.0, -1.0, -15.0);
		public static final Waypoint[] WAYPOINTS = { new Waypoint(1.2, -2.35, -90.0, -1.0*vinceHasANeed, true), // 1st
													new Waypoint(-1.4, -4.1, -180.0, 1.0*vinceHasANeed, false),
													new Waypoint(1.15, -4.25, 90.0, 1.0*vinceHasANeed, true),
													new Waypoint(-1.25, -5.35, 20.0, -1.0*vinceHasANeed, false),
													new Waypoint(-1.25, -6.45, -20.0, -1.0*vinceHasANeed, false),
													new Waypoint(1.22, -6.5, -90.0, -1.0*vinceHasANeed, true),
													new Waypoint(0.3, -8.25, -165.0, 1.0*vinceHasANeed, true),
												};
	}
	public static class PICKUP implements PATH_SET {
		private static double speedResponse = 1;
		public static final Pose START = new Pose(0.0, -0.5, 0.0); //center of start zone, bumpers approximately on edge
		/* A_RED:
		(0, 2.286)
		(-0.762, 3.81)
		(1.524, 4.572)*/
		public static final Waypoint[] A_RED = { 
													new Waypoint(0.0, -2.5, 40.0, -1.0 * speedResponse, false), //first ball
													new Waypoint(-1.0, -3.8, -45.0, -1.0 * speedResponse, false), //second ball
													new Waypoint(1.4, -4.6, -10.0, -1.0 * speedResponse, false), // third ball
													new Waypoint(1.4, -8.8, 0.0, -1.0 * speedResponse, false), //end zone
													//alternate
													// new Waypoint(0.0, -2.5, 40.0, -1.0 * speedResponse, false), //first ball
													// new Waypoint(-1.0, -3.8, 40.0, -1.0 * speedResponse, true), //second ball
													// new Waypoint(0.0, -2.5, 0.0, 1.0 * speedResponse, false), //back up instead of turning
													// new Waypoint(1.4, -4.6, -10.0, -1.0 * speedResponse, false), // third ball
													// new Waypoint(1.4, -8.8, 0.0, -1.0 * speedResponse, false), //end zone
		};
		/*A_BLUE:
		(-1.524, 4.572)
		(0.762, 5.534)
		(0, 3.858)*/
		public static final Waypoint[] A_BLUE = { 
													new Waypoint(-1.0, -2.5, 0.0, -1.0 * speedResponse, false),
													new Waypoint(-1.5, -4.6, -30.0, -1.0 * speedResponse, false), //first ball
													new Waypoint(0.0, -5.0, -90.0, -1.0 * speedResponse, false), 
													new Waypoint(1.0, -5.6, 40.0, -1.0 * speedResponse, false), //second ball
													new Waypoint(0.0, -3.9, 0.0, -1.0 * speedResponse, false), //third ball
													new Waypoint(0.0, -8.8, 0.0, -1.0 * speedResponse, false), //end zone
													//alternate
													// new Waypoint(-1.0, -2.5, 0.0, -1.0 * speedResponse, false),
													// new Waypoint(-1.5, -4.6, 45.0, -1.0 * speedResponse, false), //first ball
													// new Waypoint(0.0, -2.25, 0.0, 1.0 * speedResponse, false), //back up
													// new Waypoint(1.0, -5.6, 40.0, -1.0 * speedResponse, false), //second ball
													// new Waypoint(0.0, -3.9, 0.0, -1.0 * speedResponse, false), //third ball
													// new Waypoint(0.0, 8.8, 0.0, -1.0 * speedResponse, false), //end zone
		};
		/*B_RED:
		(0.762, 2.286)
		(-0.762, 3.81)
		(0.762, 5.534)*/
		public static final Waypoint[] B_RED = {
													new Waypoint(0.8, -2.25, 30.0, -1.0 * speedResponse, false), //first ball
													new Waypoint(-0.8, -3.8, 0.0, -1.0 * speedResponse, false), //second ball
													new Waypoint(0.8, -5.5, 0.0, -1.0 * speedResponse, false), //third ball
													new Waypoint(1.5, -8.8, -20.0, -1.0 * speedResponse, false), //end zone
		};
		/*B_BLUE:
		(-0.762, 4.572)
		(0.762, 6.069)
		(-0.762, 7.62)*/
		public static final Waypoint[] B_BLUE = {
													new Waypoint(-0.8, -4.6, -30.0, -1.0 * speedResponse, false), //first ball
													new Waypoint(0.8, 6.0, 0.0, -1.0 * speedResponse, false), //second ball
													new Waypoint(-0.8, 7.6, 45.0, -1.0 * speedResponse, false), //third ball
													new Waypoint(-1.4, -8.8, 30, -1.0 * speedResponse, false),
		};
	}

	public static final Pose OUR_GOAL = new Pose(1.71, 0.5);
	public static final Pose THEIR_GOAL = new Pose(-1.71, 16.9);

	public static final Pose PRE_TRENCH_CYCLE = new Pose(3.4, -10.0, 0.0);
	public static final Pose TRENCH_CYCLE_SHOT = new Pose(3.4, -6.0, 0.0);
	public static final double TRENCH_CUTOFF = -10.0;
	public static final double FAR_CUTOFF = -4.0;
}
