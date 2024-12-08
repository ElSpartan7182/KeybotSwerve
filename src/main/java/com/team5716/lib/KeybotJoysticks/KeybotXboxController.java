package com.team5716.lib.KeybotJoysticks;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeybotXboxController extends XboxController {

	public Trigger boton_A = new Trigger(super::getAButton);
	public Trigger boton_B = new Trigger(super::getBButton);
	public Trigger boton_X = new Trigger(super::getXButton);
	public Trigger boton_Y = new Trigger(super::getYButton);

	public Trigger boton_LeftBumper = new Trigger(super::getLeftBumper);
	public Trigger boton_RightBumper = new Trigger(super::getRightBumper);

	public Trigger boton_Start = new Trigger(super::getStartButton);
	public Trigger boton_Back = new Trigger(super::getBackButton);

	public Trigger boton_LeftStick = new Trigger(super::getLeftStickButton);
	public Trigger boton_RightStick = new Trigger(super::getRightStickButton);

	private final int D_PAD_NORTH = 0;
	private final int D_PAD_NORTH_EAST = 45;
	private final int D_PAD_EAST = 90;
	private final int D_PAD_SOUTH_EAST = 135;
	private final int D_PAD_SOUTH = 180;
	private final int D_PAD_SOUTH_WEST = 225;
	private final int D_PAD_WEST = 270;
	private final int D_PAD_NORTH_WEST = 315;

	public Trigger boton_North = new Trigger(() -> super.getPOV() == D_PAD_NORTH);
	public Trigger boton_NorthEast = new Trigger(() -> super.getPOV() == D_PAD_NORTH_EAST);
	public Trigger boton_East = new Trigger(() -> super.getPOV() == D_PAD_EAST);
	public Trigger boton_SouthEast = new Trigger(() -> super.getPOV() == D_PAD_SOUTH_EAST);
	public Trigger boton_South = new Trigger(() -> super.getPOV() == D_PAD_SOUTH);
	public Trigger boton_SouthWest = new Trigger(() -> super.getPOV() == D_PAD_SOUTH_WEST);
	public Trigger boton_West = new Trigger(() -> super.getPOV() == D_PAD_WEST);
	public Trigger boton_NorthWest = new Trigger(() -> super.getPOV() == D_PAD_NORTH_WEST);

	private double TRIGGER_PRESS_THRESHOLD = 0.5;

	public Trigger boton_LeftTrigger = new Trigger(() -> super.getLeftTriggerAxis() > TRIGGER_PRESS_THRESHOLD);
	public Trigger boton_RightTrigger = new Trigger(() -> super.getRightTriggerAxis() > TRIGGER_PRESS_THRESHOLD);

	private double LEFT_DEADBAND = 0.1;
	private double RIGHT_DEADBAND = 0.1;

	public DoubleSupplier axis_LeftX = () -> MathUtil.applyDeadband(super.getLeftX(), LEFT_DEADBAND);
	public DoubleSupplier axis_LeftY = () -> MathUtil.applyDeadband(-super.getLeftY(), LEFT_DEADBAND);
	public DoubleSupplier axis_RightX = () -> MathUtil.applyDeadband(super.getRightX(), RIGHT_DEADBAND);
	public DoubleSupplier axis_RightY = () -> MathUtil.applyDeadband(-super.getRightY(), RIGHT_DEADBAND);
	public DoubleSupplier axis_LeftTrigger = () -> super.getLeftTriggerAxis();
	public DoubleSupplier axis_RightTrigger = () -> super.getRightTriggerAxis();

	public void setTriggerPressThreshold(double threshold) {
		TRIGGER_PRESS_THRESHOLD = MathUtil.clamp(threshold, 0, 0.99);
	}
	public void setLeftDeadband(double deadband) {
		LEFT_DEADBAND = deadband;
	}
	public void setRightDeadband(double deadband) {
		RIGHT_DEADBAND = deadband;
	}
	public KeybotXboxController(int port, double leftDeadband, double rightDeadband) {
		super(port);
		LEFT_DEADBAND = leftDeadband;
		RIGHT_DEADBAND = rightDeadband;
    }
	public KeybotXboxController(int port) {
		super(port);
	}

}