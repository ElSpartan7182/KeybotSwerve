package com.team5716.lib.KeybotSwerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team5716.lib.KeybotUtils.Math.*;

public class KeybotSwerveModule extends SubsystemBase {

	// Motors //
	public TalonFX driveMotor;
	public TalonFX angleMotor;

	private CANcoder absoluteEncoder;
	private double absoluteEncoderOffset;

	public int moduleNumber;

	// Configurations //
	public static TalonFXConfiguration driveConfiguration;
	public static TalonFXConfiguration angleConfiguration;
	public static CANcoderConfiguration cancoderConfiguration;

	private DutyCycleOut driveMotorControllerOpen;
	private VelocityDutyCycle driveMotorControllerClosed;
	private PositionVoltage steerMotorController;

	public static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
	public static NeutralModeValue steerNeutralMode = NeutralModeValue.Coast;
	public static InvertedValue driveInversion = InvertedValue.CounterClockwise_Positive;
	public static InvertedValue steerInversion = InvertedValue.Clockwise_Positive;
	public static SensorDirectionValue cancoderInversion = SensorDirectionValue.CounterClockwise_Positive;
	public static String CANBusName = "";
	public static double minimumSteerSpeedPercent = 0.01;

	/* Contants */
	// L2 Module Type //
	public static double driveGearRatio = KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.driveGearRatio;
	public static double steerGearRatio = KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.angleGearRatio;
	public static double wheelCircumference = KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.wheelCircumference;
	public static double maxModuleSpeedMeters = KeybotModuleConstants.MK4i.KRAKEN_CONFIGS.L2.maxSpeedMeters;

	// Simulation //
	public static boolean isSimulation = false;
	private SwerveModuleState lastDesiredSwerveModuleState = new SwerveModuleState(0, new Rotation2d(0));
	private double desiredDrivePosition;
	private double timeFromLastUpdate = 0;
	private Timer simTimer = new Timer();
	private double lastSimTime = simTimer.get();

	public KeybotSwerveModule(int moduleNumber, int driveMotorID, int steerMotorID, int absoluteEncoderID,
			double absoluteEncoderOffset) {

		simTimer.start();

		this.moduleNumber = moduleNumber;

		driveMotor = new TalonFX(driveMotorID, CANBusName);
		angleMotor = new TalonFX(steerMotorID, CANBusName);
		driveMotorControllerClosed = new VelocityDutyCycle(0);
		driveMotorControllerOpen = new DutyCycleOut(0);
		steerMotorController = new PositionVoltage(0);

		absoluteEncoder = new CANcoder(absoluteEncoderID, CANBusName);
		this.absoluteEncoderOffset = absoluteEncoderOffset;

		driveConfiguration = new TalonFXConfiguration();
		angleConfiguration = new TalonFXConfiguration();
		cancoderConfiguration = new CANcoderConfiguration();
	}

	public void configure() {
		// Drive Motor Config //
		driveConfiguration.MotorOutput.Inverted = driveInversion;
		driveConfiguration.MotorOutput.NeutralMode = driveNeutralMode;
		driveConfiguration.Feedback.SensorToMechanismRatio = driveGearRatio;

		driveMotor.getConfigurator().apply(driveConfiguration);

		// Steer Motor Config //
		angleConfiguration.MotorOutput.Inverted = steerInversion;
		angleConfiguration.MotorOutput.NeutralMode = steerNeutralMode;
		angleConfiguration.Feedback.SensorToMechanismRatio = steerGearRatio;
		angleConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

		angleMotor.getConfigurator().apply(angleConfiguration);

		// Absolute Encoder Config //
		cancoderConfiguration.MagnetSensor.SensorDirection = cancoderInversion;
		absoluteEncoder.getConfigurator().apply(cancoderConfiguration);
	}

	public double getRawAbsoluteEncoder() {
		return absoluteEncoder.getAbsolutePosition().getValue();
	}

	public double getAbsoluteEncoder() {
		double rotations = getRawAbsoluteEncoder();

		rotations -= absoluteEncoderOffset;

		return rotations;
	}

	public void resetSteerMotorToAbsolute() {
		angleMotor.setPosition(getAbsoluteEncoder());
	}

	public void resetDriveMotorEncoder() {
		driveMotor.setPosition(0);
	}

	public SwerveModuleState getActualModuleState() {

		double velocity = KeybotConversions.rotationsToMeters(driveMotor.getVelocity().getValue(), wheelCircumference, 1);

		Rotation2d angle = Rotation2d.fromDegrees(Units.rotationsToDegrees(angleMotor.getPosition().getValue()));

		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModuleState getDesiredModuleState() {
		return lastDesiredSwerveModuleState;
	}

	public SwerveModulePosition getModulePosition() {
		if (isSimulation) {
			timeFromLastUpdate = simTimer.get() - lastSimTime;
			lastSimTime = simTimer.get();
			desiredDrivePosition += (lastDesiredSwerveModuleState.speedMetersPerSecond * timeFromLastUpdate);

			return new SwerveModulePosition(desiredDrivePosition, lastDesiredSwerveModuleState.angle);
		}

		double distance = KeybotConversions.rotationsToMeters(driveMotor.getPosition().getValue(), wheelCircumference, 1);

		Rotation2d angle = Rotation2d.fromDegrees(Units.rotationsToDegrees(angleMotor.getPosition().getValue()));

		return new SwerveModulePosition(distance, angle);
	}

	public void neutralDriveOutput() {
		driveMotor.setControl(new NeutralOut());
	}

	public void setModuleState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace) {

		SwerveModuleState state = KeybotSwerveStates.optimize(desiredState, getActualModuleState().angle);
		lastDesiredSwerveModuleState = state;

		if (isOpenLoop) {
			driveMotorControllerOpen.Output = (state.speedMetersPerSecond / maxModuleSpeedMeters);
			driveMotor.setControl(driveMotorControllerOpen);

		} else {
			driveMotorControllerClosed.Velocity = KeybotConversions.metersToRotations(state.speedMetersPerSecond,
					wheelCircumference, 1);
			driveMotor.setControl(driveMotorControllerClosed);
		}

		double rotation = state.angle.getRotations();

		if (Math.abs(state.speedMetersPerSecond) < (minimumSteerSpeedPercent * maxModuleSpeedMeters) && !steerInPlace) {
			return;
		}

		steerMotorController.Position = rotation;
		angleMotor.setControl(steerMotorController);
	}
}