package com.team5716.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team5716.lib.KeybotSwerve.KeybotModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final Measure<Voltage> VOLTAGE_MAX = Units.Volts.of(12);

    public static class swerveConstants {

    public static final double FL_OFFSET = 0.100098;
    public static final double FR_OFFSET = -0.348633;
    public static final double BL_OFFSET = 0.176270;
    public static final double BR_OFFSET = -0.106201;

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final Measure<Distance> WHEEL_DIAMETER = Units.Inches.of(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.in(Units.Meters) * Math.PI;

    public static final double THEORETICAL_MAX_DRIVE_SPEED = KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.maxSpeedMeters;

    public static final Measure<Velocity<Distance>> DRIVE_SPEED = Units.FeetPerSecond.of(15.1);
    
    public static final double TRACK_WIDTH = Units.Meters.convertFrom(19.75, Units.Inches);
    
    public static final double WHEELBASE = Units.Meters.convertFrom(19.75, Units.Inches);

    public static final KeybotModuleConstants SWERVE_CONSTANTS = new KeybotModuleConstants(
        KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.angleGearRatio,
        WHEEL_CIRCUMFERENCE,
        KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.driveGearRatio,
        KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.maxSpeedMeters);

    public static final Measure<Angle> ROTATION_TOLERANCE = Units.Degrees.of(5);

    public static final boolean DRIVE_ENABLE_CURRENT_LIMITING = true;
    public static final double DRIVE_CURRENT_THRESH = 40;
    public static final double DRIVE_CURRENT_LIMIT = 30;
    public static final double DRIVE_CURRENT_TIME_THRESH = 0.1;

    public static final boolean ANGLE_ENABLE_CURRENT_LIMITING = true;
    public static final double ANGLE_CURRENT_THRESH = 40;
    public static final double ANGLE_CURRENT_LIMIT = 30;
    public static final double ANGLE_CURRENT_TIME_THRESH = 0.1;

  }

  public static class fieldConstants {

    public static Optional<Alliance> ALLIANCE = Optional.empty();
    public static Measure<Distance> FIELD_LENGTH = Units.Meters.of(16.541);

    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    public static Supplier<Pose3d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> new Pose3d[] { redConstants.SPEAKER_CENTER, redConstants.AMP, redConstants.SOURCE,
            redConstants.LEFT_STAGE,
            redConstants.CENTER_STAGE, redConstants.RIGHT_STAGE, redConstants.SUBWOOFER, redConstants.SHUFFLE };

      }
      return () -> new Pose3d[] { blueConstants.SPEAKER_CENTER, blueConstants.AMP, blueConstants.SOURCE,
          blueConstants.LEFT_STAGE,
          blueConstants.CENTER_STAGE, blueConstants.RIGHT_STAGE, blueConstants.SUBWOOFER, blueConstants.SHUFFLE };
    }
  }

   private static final class blueConstants {

      private static final Pose3d SPEAKER_CENTER = new Pose3d(0.457 / 2, 5.557034, 2.105 - (0.133 / 2),
          new Rotation3d(0, 0, 0));

      private static final Pose3d AMP = new Pose3d(1.827, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));
      
      private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(300)));
      private static final Pose3d LEFT_STAGE = new Pose3d(
          new Pose2d(4.541771411895752, 4.736017227172852, Rotation2d.fromDegrees(300)));
      private static final Pose3d CENTER_STAGE = new Pose3d(
          new Pose2d(5.554078578948975, 4.124814033508301, Rotation2d.fromDegrees(180)));
      private static final Pose3d RIGHT_STAGE = new Pose3d(
          new Pose2d(4.524875164031982, 3.488827705383301, Rotation2d.fromDegrees(60)));

      private static final Pose3d SUBWOOFER = new Pose3d(new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(0)));

      private static final Pose3d SHUFFLE = new Pose3d(
          new Pose2d(3.42, 6.08, Rotation2d.fromDegrees(0)));

      private static final Measure<Distance> WING_LINE_X = Units.Meters.of(6.3);
    }

    private static final class redConstants {
      private static final Pose3d SPEAKER_CENTER = new Pose3d(Units.Meters.of(16.541).in(Units.Meters) - (0.457 / 2), 5.557034,
          2.105 - (0.133 / 2),
          new Rotation3d(0, 0, 0));

      private static final Pose3d AMP = new Pose3d(14.706, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));

      private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(240)));
      private static final Pose3d LEFT_STAGE = new Pose3d(
          new Pose2d(12.0610990524292, 3.4952545166015625, Rotation2d.fromDegrees(120)));
      private static final Pose3d CENTER_STAGE = new Pose3d(
          new Pose2d(10.983105659484863, 4.096443176269531, Rotation2d.fromDegrees(0)));
      private static final Pose3d RIGHT_STAGE = new Pose3d(
          new Pose2d(12.021082878112793, 4.7371745109558105, Rotation2d.fromDegrees(240)));

      private static final Pose3d SUBWOOFER = new Pose3d(
          new Pose2d(Units.Meters.of(16.541).in(Units.Meters) - 1.35, 5.50, Rotation2d.fromDegrees(180)));
      private static final Pose3d SHUFFLE = new Pose3d(
          new Pose2d(Units.Meters.of(16.541).in(Units.Meters) - 3.42, 6.08, Rotation2d.fromDegrees(0)));

      private static final Measure<Distance> WING_LINE_X = Units.Meters.of(16.541).minus(Units.Meters.of(6.3));
    }
  }