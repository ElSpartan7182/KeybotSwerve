package com.team5716.lib.KeybotSwerve;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KeybotSwerve extends SubsystemBase {
	public KeybotSwerveModule[] modules;
	public SwerveDrivePoseEstimator swervePoseEstimator;
	public SwerveDriveKinematics swerveKinematics;
	public Pigeon2 pigeon;
	public boolean isFieldRelative;

	public KeybotModuleConstants swerveConstants;
	
	private double driveBaseRadius;
	public PIDConstants autoDrivePID;
	public PIDConstants autoAnglePID;
	private Matrix<N3, N1> stateStdDevs;
	private Matrix<N3, N1> visionStdDevs;
	public HashMap<String, Command> autoEventMap = new HashMap<>();
	public ReplanningConfig autoReplanningConfig;
	public BooleanSupplier autoFlipPaths;

	public PathPlannerTrajectory exampleAuto;

	private boolean isSimulation;
	public double simAngle = 0;
	private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[]{new SwerveModuleState(),
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
	public double timeFromLastUpdate = 0;
	public double lastSimulationTime = Timer.getFPGATimestamp();
	public Field2d field;

	
	public KeybotSwerve(KeybotModuleConstants swerveConstants, KeybotSwerveModule[] modules, double wheelbase,
			double trackWidth, String CANBusName, int pigeonCANId, double minimumSteerPercent,
			InvertedValue driveInversion, InvertedValue angleInversion, SensorDirectionValue cancoderInversion,
			NeutralModeValue driveNeutralMode, NeutralModeValue steerNeutralMode, Matrix<N3, N1> stateStdDevs,
			Matrix<N3, N1> visionStdDevs, PIDConstants autoDrivePID, PIDConstants autoAnglePID,
			ReplanningConfig autoReplanningConfig, BooleanSupplier autoFlipPaths, boolean isSimulation) {

		isFieldRelative = true;
		field = new Field2d();

		// Location of all modules in the WPILib robot coordinate system
		swerveKinematics = new SwerveDriveKinematics(new Translation2d(wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelbase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));

		this.modules = modules;
		this.stateStdDevs = stateStdDevs;
		this.visionStdDevs = visionStdDevs;
		this.swerveConstants = swerveConstants;
		this.autoDrivePID = autoDrivePID;
		this.autoAnglePID = autoAnglePID;
		this.isSimulation = isSimulation;
		this.autoReplanningConfig = autoReplanningConfig;
		this.autoFlipPaths = autoFlipPaths;

		KeybotSwerveModule.isSimulation = isSimulation;
		KeybotSwerveModule.wheelCircumference = swerveConstants.wheelCircumference;
		KeybotSwerveModule.maxModuleSpeedMeters = swerveConstants.maxSpeedMeters;
		KeybotSwerveModule.driveGearRatio = swerveConstants.driveGearRatio;
		KeybotSwerveModule.steerGearRatio = swerveConstants.angleGearRatio;

		KeybotSwerveModule.CANBusName = CANBusName;
		KeybotSwerveModule.minimumSteerSpeedPercent = minimumSteerPercent;

		KeybotSwerveModule.driveInversion = driveInversion;
		KeybotSwerveModule.driveNeutralMode = driveNeutralMode;

		KeybotSwerveModule.steerInversion = angleInversion;
		KeybotSwerveModule.steerNeutralMode = steerNeutralMode;
		KeybotSwerveModule.cancoderInversion = cancoderInversion;

		driveBaseRadius = Math.sqrt(Math.pow((wheelbase / 2), 2) + Math.pow((trackWidth / 2), 2));
		pigeon = new Pigeon2(pigeonCANId, CANBusName);

		Timer.delay(1.5);
		resetModulesToAbsolute();
		configure();

		AutoBuilder.configureHolonomic(this::getPose, this::resetPoseToPose, this::getChassisSpeeds,
				this::driveAutonomous, new HolonomicPathFollowerConfig(autoDrivePID, autoAnglePID,
						swerveConstants.maxSpeedMeters, driveBaseRadius, autoReplanningConfig),
				autoFlipPaths, this);
	}

	public void configure() {
		for (KeybotSwerveModule mod : modules) {
			mod.configure();
		}

		swervePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getGyroRotation(), getModulePositions(),
				new Pose2d(), stateStdDevs, visionStdDevs);

	}

	public void resetModulesToAbsolute() {
		for (KeybotSwerveModule mod : modules) {
			mod.resetSteerMotorToAbsolute();
		}
	}
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (KeybotSwerveModule mod : modules) {
			positions[mod.moduleNumber] = mod.getModulePosition();
		}

		return positions;
	}

	public SwerveModuleState[] getActualModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (KeybotSwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getActualModuleState();
		}

		return states;
	}

	public SwerveModuleState[] getDesiredModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (KeybotSwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getDesiredModuleState();
		}

		return lastDesiredStates;
	}

	public ChassisSpeeds getChassisSpeeds() {
		return swerveKinematics.toChassisSpeeds(getActualModuleStates());
	}

	public void setModuleStates(SwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, swerveConstants.maxSpeedMeters);
		lastDesiredStates = desiredModuleStates;

		for (KeybotSwerveModule mod : modules) {
			mod.setModuleState(desiredModuleStates[mod.moduleNumber], isOpenLoop, false);
		}
	}
	public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
		ChassisSpeeds chassisSpeeds;

		if (isFieldRelative) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					getRotation());
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		SwerveModuleState[] desiredModuleStates = swerveKinematics
				.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
		setModuleStates(desiredModuleStates, isOpenLoop);
	}
	public void driveAutonomous(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] desiredModuleStates = swerveKinematics
				.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
		setModuleStates(desiredModuleStates, false);
	}

	public void neutralDriveOutputs() {
		for (KeybotSwerveModule mod : modules) {
			mod.neutralDriveOutput();
		}
	}

	public void setFieldRelative() {
		isFieldRelative = true;
	}
	public void setRobotRelative() {
		isFieldRelative = false;
	}

	public void updatePoseEstimator() {
		swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation(), getModulePositions());
	}
	public Pose2d getPose() {
		return swervePoseEstimator.getEstimatedPosition();
	}
	public Rotation2d getRotation() {
		return swervePoseEstimator.getEstimatedPosition().getRotation();
	}
	public void resetPoseToPose(Pose2d pose) {
		swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
	}
    public void resetHeading(double deg) {
        Pose2d actualOdometry = getPose();
        Rotation2d rotation = new Rotation2d(Math.toDegrees(deg)); 
        Pose2d newOdometry = new Pose2d(actualOdometry.getX(), actualOdometry.getY(), rotation);
        resetPoseToPose(newOdometry);
    }
	private Rotation2d getGyroRotation() {
		if (isSimulation && lastDesiredStates != null) {
			simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond * timeFromLastUpdate;
			simAngle = simAngle % (2 * Math.PI);
			simAngle = (simAngle < 0) ? simAngle + (2 * Math.PI) : simAngle;
			return Rotation2d.fromRadians(simAngle);
		}
		double yaw = pigeon.getYaw().getValueAsDouble() % 360;
		return (yaw < 0) ? Rotation2d.fromDegrees(yaw + 360) : Rotation2d.fromDegrees(yaw);
	}
	public double getGyroRate() {
		return pigeon.getRate();
	}
	public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
		swervePoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
	}
	public void updateTimer() {
		timeFromLastUpdate = Timer.getFPGATimestamp() - lastSimulationTime;
		lastSimulationTime = Timer.getFPGATimestamp();
	}

	@Override
	public void periodic() {
		updateTimer();
		updatePoseEstimator();
	}
}