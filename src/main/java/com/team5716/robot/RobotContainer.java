// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5716.robot;

import com.team5716.lib.KeybotJoysticks.KeybotXboxController;
import com.team5716.robot.Commands.Drive;
import com.team5716.robot.Subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Xbox Controller //
  private final KeybotXboxController KEYBOT_DRIVE_CONTROLLER = new KeybotXboxController(0);

  // False trigger //
  private final Trigger falseTrigger = new Trigger(() -> false);


  private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public RobotContainer() {
    configureDriver();
  }

  private void configureDriver() {

     swerveSubsystem.setDefaultCommand(
              new Drive(swerveSubsystem, KEYBOT_DRIVE_CONTROLLER.axis_LeftX, KEYBOT_DRIVE_CONTROLLER.axis_LeftX,
                  KEYBOT_DRIVE_CONTROLLER.axis_RightX,
                  KEYBOT_DRIVE_CONTROLLER.btn_RightBumper, falseTrigger, falseTrigger,
                  falseTrigger, falseTrigger, falseTrigger,
                  falseTrigger, falseTrigger));
    KEYBOT_DRIVE_CONTROLLER.btn_Y.onTrue(new InstantCommand(()->swerveSubsystem.resetHeading(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
