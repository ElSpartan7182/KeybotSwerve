package com.team5716.robot.Subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.team5716.robot.Preferences;
import com.team5716.lib.KeybotSwerve.KeybotSwerve;
import com.team5716.lib.KeybotSwerve.KeybotSwerveModule;
import com.team5716.robot.Constants;
import com.team5716.robot.Robot;
import com.team5716.robot.Preferences.swervePreferences;
import com.team5716.robot.Swerve.SwerveMap;

public class SwerveSubsystem extends KeybotSwerve {
  private static TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
  private static TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();
  private static PIDController yawSnappingController;
  private static String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };

  StructPublisher<Pose2d> pose2dPublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Swerve/Pose2d", Pose2d.struct).publish();
  StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/Swerve/DesiredState", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> actualStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/Swerve/ActualState", SwerveModuleState.struct).publish();

  private static KeybotSwerveModule[] modules = new KeybotSwerveModule[] {
      new KeybotSwerveModule(0, SwerveMap.DriveTrainMap.FL_DRIVE, SwerveMap.DriveTrainMap.FL_ANGLE,
          SwerveMap.DriveTrainMap.FL_CANCODER, Constants.swerveConstants.FL_OFFSET),
      new KeybotSwerveModule(1, SwerveMap.DriveTrainMap.FR_DRIVE, SwerveMap.DriveTrainMap.FR_ANGLE,
          SwerveMap.DriveTrainMap.FR_CANCODER, Constants.swerveConstants.FR_OFFSET),
      new KeybotSwerveModule(2, SwerveMap.DriveTrainMap.BL_DRIVE, SwerveMap.DriveTrainMap.BL_ANGLE,
          SwerveMap.DriveTrainMap.BL_CANCODER, Constants.swerveConstants.BL_OFFSET),
      new KeybotSwerveModule(3, SwerveMap.DriveTrainMap.BR_DRIVE, SwerveMap.DriveTrainMap.BR_ANGLE,
          SwerveMap.DriveTrainMap.BR_CANCODER, Constants.swerveConstants.BR_OFFSET),
  };

  public SwerveSubsystem() {
    super(
        Constants.swerveConstants.SWERVE_CONSTANTS,
        modules,
        Constants.swerveConstants.WHEELBASE,
        Constants.swerveConstants.TRACK_WIDTH,
        SwerveMap.DriveTrainMap.CANBUS,
        SwerveMap.DriveTrainMap.PIGEON_ID,
        swervePreferences.minimumSteerSpeedPercent.getValue(),
        Constants.swerveConstants.DRIVE_MOTOR_INVERT,
        Constants.swerveConstants.ANGLE_MOTOR_INVERT,
        Constants.swerveConstants.CANCODER_INVERT,
        Constants.swerveConstants.DRIVE_NEUTRAL_MODE,
        Constants.swerveConstants.ANGLE_NEUTRAL_MODE,
        VecBuilder.fill(
            swervePreferences.measurementStdDevsPosition.getValue(),
            swervePreferences.measurementStdDevsPosition.getValue(),
            swervePreferences.measurementStdDevsHeading.getValue()),
        VecBuilder.fill(
            Preferences.visionPreferences.multiTagStdDevsPosition.getValue(),
            Preferences.visionPreferences.multiTagStdDevsPosition.getValue(),
            Preferences.visionPreferences.multiTagStdDevsHeading.getValue()),
        new PIDConstants(
            swervePreferences.driveAutoP,
            swervePreferences.driveAutoI,
            swervePreferences.driveAutoD),
        new PIDConstants(
            swervePreferences.angleAutoP,
            swervePreferences.angleAutoI,
            swervePreferences.angleAutoD),
        new ReplanningConfig(true, true),
        () -> Constants.fieldConstants.isRedAlliance(),
        Robot.isSimulation());
  }

  @Override
  public void configure() {
    driveConfiguration.Slot0.kP = swervePreferences.driveP.getValue();
    driveConfiguration.Slot0.kI = swervePreferences.driveI.getValue();
    driveConfiguration.Slot0.kD = swervePreferences.driveD.getValue();

    driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = Constants.swerveConstants.DRIVE_ENABLE_CURRENT_LIMITING;
    driveConfiguration.CurrentLimits.SupplyCurrentThreshold = Constants.swerveConstants.DRIVE_CURRENT_THRESH;
    driveConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.swerveConstants.DRIVE_CURRENT_LIMIT;
    driveConfiguration.CurrentLimits.SupplyTimeThreshold = Constants.swerveConstants.DRIVE_CURRENT_TIME_THRESH;

    driveConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    driveConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
    driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    angleConfiguration.Slot0.kP = swervePreferences.angleP.getValue();
    angleConfiguration.Slot0.kI = swervePreferences.angleI.getValue();
    angleConfiguration.Slot0.kD = swervePreferences.angleD.getValue();

    angleConfiguration.CurrentLimits.SupplyCurrentLimitEnable = Constants.swerveConstants.ANGLE_ENABLE_CURRENT_LIMITING;
    angleConfiguration.CurrentLimits.SupplyCurrentThreshold = Constants.swerveConstants.ANGLE_CURRENT_THRESH;
    angleConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.swerveConstants.ANGLE_CURRENT_LIMIT;
    angleConfiguration.CurrentLimits.SupplyTimeThreshold = Constants.swerveConstants.ANGLE_CURRENT_TIME_THRESH;

    angleConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    angleConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
    angleConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    KeybotSwerveModule.driveConfiguration = driveConfiguration;
    KeybotSwerveModule.angleConfiguration = angleConfiguration;

    yawSnappingController = new PIDController(
        swervePreferences.yawSnapP,
        swervePreferences.yawSnapI,
        swervePreferences.yawSnapD);
    yawSnappingController.enableContinuousInput(0, 360);
    yawSnappingController.setTolerance(Constants.swerveConstants.ROTATION_TOLERANCE.in(Units.Degrees));

    super.configure();
  }

  public void addEventToAutoMap(String key, Command command) {
    super.autoEventMap.put(key, command);
  }
  public Measure<Velocity<Angle>> getVelocityToSnap(Rotation2d desiredYaw) {
    double yawSetpoint = yawSnappingController.calculate(getRotation().getDegrees(),
        desiredYaw.getDegrees());

    yawSetpoint = MathUtil.clamp(yawSetpoint, -swervePreferences.maxTurnSpeed.in(Units.DegreesPerSecond),
        swervePreferences.maxTurnSpeed.in(Units.DegreesPerSecond));

    return Units.DegreesPerSecond.of(yawSetpoint);
  }

  public Measure<Velocity<Angle>> getVelocityToSnap(Measure<Angle> desiredYaw) {
    return getVelocityToSnap(Rotation2d.fromDegrees(desiredYaw.in(Units.Degrees)));
  }

  public boolean isDrivetrainAtAngle(Rotation2d desiredAngle) {
    return (Math.abs(getRotation().getDegrees() - desiredAngle.getDegrees()) 
    < Constants.swerveConstants.ROTATION_TOLERANCE.in(Units.Degrees));
  }

  public Rotation2d getAngleToTarget(Pose2d targetPose) {
    Pose2d robotPose = getPose();

    Pose2d relativeToTarget = robotPose.relativeTo(targetPose);

    Rotation2d desiredLockingAngle = new Rotation2d(relativeToTarget.getX(), relativeToTarget.getY());

    SmartDashboard.putNumber("ANGLE TO TARGET", desiredLockingAngle.getDegrees());

    return desiredLockingAngle;
  }
  public double getGyroRate() {
    return pigeon.getRate();
  }

  public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
    swervePoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
  }

  @Override
  public void periodic() {
    super.periodic();

    for (KeybotSwerveModule mod : modules) {
      SmartDashboard.putNumber("Swerve/Module " + moduleNames[mod.moduleNumber] + "/Desired Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getDesiredModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));
      SmartDashboard.putNumber("Swerve/Module " + moduleNames[mod.moduleNumber] + "/Actual Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getActualModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));

      SmartDashboard.putNumber("Swerve/Module " + moduleNames[mod.moduleNumber] + "/Desired Angle (Degrees)",
          Math.abs(
              Units.Meters.convertFrom(getDesiredModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));
      SmartDashboard.putNumber("Swerve/Module " + moduleNames[mod.moduleNumber] + "/Actual Angle (Degrees)",
          Math.abs(Units.Meters.convertFrom(getActualModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));

      SmartDashboard.putNumber(
          "Swerve/Module " + moduleNames[mod.moduleNumber] + "/Offset Absolute Encoder Angle (Rotations)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber(
          "Swerve/Module " + moduleNames[mod.moduleNumber] + "/Absolute Encoder Raw Value (Rotations)",
          mod.getRawAbsoluteEncoder());
    }

    pose2dPublisher.set(getPose());
    desiredStatePublisher.set(getDesiredModuleStates());
    actualStatePublisher.set(getActualModuleStates());

    SmartDashboard.putNumber("Swerve Rotation", getRotation().getDegrees());
  }
}