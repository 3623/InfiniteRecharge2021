package frc.util;

import java.util.LinkedList;

public class Utils {

	public static Boolean withinThreshold(double value, double goal, double epsilon) {
		return Math.abs(value - goal) <= epsilon;
	}

	public static Boolean outsideDeadband(double value, double center, double deadband) {
		return !withinThreshold(value, center, deadband);
	}

	public static Tuple limitTuple(Tuple tuple, double upperBound, double lowerBound) {
		tuple.left = limit(tuple.left, upperBound, lowerBound);
		tuple.right = limit(tuple.right, upperBound, lowerBound);
		return tuple;
	}

	public static double limit(double value, double upperBound, double lowerBound) {
		return Math.max(lowerBound, Math.min(upperBound, value));
	}

	public static double limit(double value) {
		return limit(value, 1.0, -1.0);
	}

	public static double applyDeadband(double value, double deadband) {
		if (Utils.withinThreshold(value, 0.0, deadband)) {
			return 0.0;
		} else {
			return value;
		}
	}

	public static void main(String[] args) {
		System.out.println();
	}

	public static class MovingAverage {


 		private int numSamples;
		private LinkedList<Double> samples;
		private double sum = 0.0;

		public MovingAverage(int numSamples) {
			this.numSamples = numSamples;
			samples = new LinkedList<>();
			for (int i = 0; i < numSamples; i++) samples.add(0.0);
		}

		public double update(double newSample) {
			sum += newSample;
			sum -= samples.removeFirst();
			return sum / numSamples;
		}
	}
}