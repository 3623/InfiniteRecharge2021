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
													new Waypoint(1.0, -3.0, 60.0, 0.8, true),
													new Waypoint(0.1, -6.7, -5.0, -0.8, true),
													new Waypoint(1.0, -3.0, 15.0, 0.8, true)};
	}

	public static class MIDDLE implements PATH_SET{
		public static final Pose START = new Pose(0.5, -3.5, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(0.2, -6.7, -15.0, -0.8, true),
													new Waypoint(1.0, -3.0, 15.0, 0.8, true)};
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
													new Waypoint(3.4, -8.5, 5.0, 1.0, false),
													new Waypoint(2.9, -5.0, 30.0, 1.0, true),
													new Waypoint(0.0, -6.7, 70.0, -1.0, true),
													new Waypoint(2.9, -5.3, 70.0, 1.0, true)};
	}

	public static class STEAL implements PATH_SET {
		public static final Pose START = new Pose(2.5, -3.0, 0.0);
		public static final Waypoint[] WAYPOINTS = {new Waypoint(1.6, -5.9, 70.0, -1.0, true),
													new Waypoint(-1.6, -8.8, -20.0, -1.0, false),
													new Waypoint(-0.6, -9.6, -70.0, -1.0, false),
													new Waypoint(1.0, -9.5, -110.0, -1.0, false),
													new Waypoint(2.0, -4.5, -180.0, -1.0, true)};
	}

	public static final Pose OUR_GOAL = new Pose(1.71, 0.5);
	public static final Pose THEIR_GOAL = new Pose(-1.71, 16.9);

	public static final Pose PRE_TRENCH_CYCLE = new Pose(3.4, -10.0, 0.0);
	public static final Pose TRENCH_CYCLE_SHOT = new Pose(3.4, -6.0, 0.0);
	public static final double TRENCH_CUTOFF = -10.0;
	public static final double FAR_CUTOFF = -4.0;
}
