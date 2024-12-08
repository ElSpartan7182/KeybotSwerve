package com.team5716.lib.KeybotSwerve;

import edu.wpi.first.math.util.Units;

public class KeybotModuleConstants {
    private static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
    private static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;
    public double angleGearRatio;
    public double wheelCircumference;
    public double driveGearRatio;
    public double maxSpeedMeters;

    public KeybotModuleConstants(
        double angleGearRatio, 
        double wheelCircumference, 
        double driveGearRatio,
        double maxSpeedMeters
        ) {    
            this.angleGearRatio = angleGearRatio;
            this.wheelCircumference = wheelCircumference;
            this.driveGearRatio = driveGearRatio;
            this.maxSpeedMeters = maxSpeedMeters;
    }
    public static class MK4i {
        public static class KRAKEN_CONFIGS {
            public static final KeybotModuleConstants L1 = new KeybotModuleConstants(ANGLE_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                8.14, Units.feetToMeters(12.9));
            public static final KeybotModuleConstants L2 = new KeybotModuleConstants(ANGLE_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                6.75, Units.feetToMeters(15.5));
            public static final KeybotModuleConstants L3 = new KeybotModuleConstants(ANGLE_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
                6.12, Units.feetToMeters(17.1));
        }
    }
}
