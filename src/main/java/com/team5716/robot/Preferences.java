package com.team5716.robot;

import com.team5716.lib.KeybotPreferences.DoublePreferences;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;


public class Preferences {
    public static final class swervePreferences {
    public static final DoublePreferences minimumSteerSpeedPercent = new DoublePreferences(
        "minimumSteerSpeed", 0.01);

    public static final DoublePreferences driveSpeed = new DoublePreferences("driveSpeed",
        Constants.swerveConstants.DRIVE_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond));

    public static final Measure<Velocity<Angle>> maxManualTurnSpeed = edu.wpi.first.units.Units.DegreesPerSecond.of(30);
    public static final Measure<Velocity<Angle>> maxTurnSpeed = edu.wpi.first.units.Units.DegreesPerSecond.of(520 * 2);

    public static final DoublePreferences slowModeMultiplier = new DoublePreferences("slowModeMultiplier", .5);

    public static final DoublePreferences measurementStdDevsPosition = new DoublePreferences(
        "measurementStdDevsPosition", 0.05);

    public static final DoublePreferences measurementStdDevsHeading = new DoublePreferences(
        "measurementStdDevsHeading", edu.wpi.first.units.Units.Radians.convertFrom(5, edu.wpi.first.units.Units.Degrees));

    public static final DoublePreferences driveP = new DoublePreferences("driveP", 0.18);
    public static final DoublePreferences driveI = new DoublePreferences("driveI", 0.0);
    public static final DoublePreferences driveD = new DoublePreferences("driveD", 0);

    public static final DoublePreferences angleP = new DoublePreferences("angleP", 100);
    public static final DoublePreferences angleI = new DoublePreferences("angleI", 0.0);
    public static final DoublePreferences angleD = new DoublePreferences("angleD", 0.0);

    public static final DoublePreferences driveKs = new DoublePreferences("driveKs", 0);
    public static final DoublePreferences driveKa = new DoublePreferences("driveKa", 0);
    public static final DoublePreferences driveKv = new DoublePreferences("driveKv", (1 / driveSpeed.getValue()));

    public static final double driveAutoP = 5;
    public static final double driveAutoI = 0;
    public static final double driveAutoD = 0;

    public static final double angleAutoP = 2.5;
    public static final double angleAutoI = 0;
    public static final double angleAutoD = 0;

    public static final double yawSnapP = 9;
    public static final double yawSnapI = 0;
    public static final double yawSnapD = 1;
  }

  public static final class visionPreferences {
    public static final DoublePreferences multiTagStdDevsPosition = new DoublePreferences(
        "multiTagStdDevsPosition", 0.7);

    public static final DoublePreferences multiTagStdDevsHeading = new DoublePreferences(
        "multiTagStdDevsHeading", 9999999);
  }
}
