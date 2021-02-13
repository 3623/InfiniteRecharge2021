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
		public static final Pose START = new Pose(0.0, -1.0, -10.0);
		public static final Waypoint[] WAYPOINTS = { new Waypoint(-1.0, -4.0, 90.0, -1.0*0.3, false), // 1st
													new Waypoint(-1.5, -3.5, -180.0, -1.0*0.3, false),
													new Waypoint(-1.0, -3.0, -90.0, -1.0*0.3, false),
													new Waypoint(1.0, -6.5, -90.0, -1.0*0.3, false), // 2nd
													new Waypoint(1.5, -6.0, 180.0, -1.0*0.3, false),
													new Waypoint(1.0, -5.5, 90.0, -1.0*0.3, false),
													new Waypoint(-1.5, -7.6, 0.0, -1.0*0.3, false), // 3rd
													new Waypoint(-1.0, -8.1, -90.0, -1.0*0.3, false),
													// new Waypoint(-0.5, -7.6, 180.0, -1.0, false),
													new Waypoint(0.0, -1.0, -170.0, -1.0*0.3, false) };
	}

	public static class SLALOM implements PATH_SET {
		public static final Pose START = new Pose(-1.0, -1.0, -25.0);
		public static final Waypoint[] WAYPOINTS = { new Waypoint(1.0, -3.0, -30.0, -1.0, false), // 1st
													new Waypoint(0.0, -6.5, 45.0, -1.0, false),
													new Waypoint(-1.0, -7.5, 0.0, -1.0, false),
													new Waypoint(0.0, -8.5, -90.0, -1.0, false),
													new Waypoint(1.0, -7.5, 180.0, -1.0, false),
													new Waypoint(0.0, -6.5, 135.0, -1.0, false),
													new Waypoint(0.0, -2.25, -135.0, -1.0, false),
													new Waypoint(1.0, -1.0, -165.0, -1.0, false),};
	}

	public static class BOUNCE implements PATH_SET {
		public static final Pose START = new Pose(0.0, -1.0, -15.0);
		public static final Waypoint[] WAYPOINTS = { new Waypoint(1.2, -2.25, -90.0, -1.0*0.3, true), // 1st
													new Waypoint(-1.1, -3.0, -180.0, 1.0*0.3, false),
													new Waypoint(1.2, -3.8, 90.0, 1.0*0.3, true),
													new Waypoint(-.95, -4.5, 20.0, -1.0*0.3, false),
													new Waypoint(-.95, -5.4, -20.0, -1.0*0.3, false),
													new Waypoint(1.2, -6.1, -90.0, -1.0*0.3, true),
													new Waypoint(0.0, -8.25, -165.0, 1.0*0.3, true),
												};
	}

	public static final Pose OUR_GOAL = new Pose(1.71, 0.5);
	public static final Pose THEIR_GOAL = new Pose(-1.71, 16.9);

	public static final Pose PRE_TRENCH_CYCLE = new Pose(3.4, -10.0, 0.0);
	public static final Pose TRENCH_CYCLE_SHOT = new Pose(3.4, -6.0, 0.0);
	public static final double TRENCH_CUTOFF = -10.0;
	public static final double FAR_CUTOFF = -4.0;
}
