package com.team5716.lib.KeybotUtils.Math;

public class KeybotConversions {
    public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
		double wheelRPS = rotations / gearRatio;
		double wheelMPS = (wheelRPS * circumference);
		return wheelMPS;
	}
	public static double metersToRotations(double meters, double circumference, double gearRatio) {
		double wheelRotations = meters / circumference;
		double motorRotations = wheelRotations * gearRatio;

		return motorRotations;
	}
}
