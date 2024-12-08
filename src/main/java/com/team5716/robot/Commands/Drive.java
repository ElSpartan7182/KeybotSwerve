package com.team5716.robot.Commands;

import java.util.function.DoubleSupplier;

import com.team5716.robot.Constants;
import com.team5716.robot.Preferences.swervePreferences;
import com.team5716.robot.Subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Drive extends Command {
  SwerveSubsystem keybotSwerve;
  DoubleSupplier xAxis, yAxis, rAxis;
  boolean isOpenLoop;
  Trigger slowModeMultiplier, north, south, east, west, chain, source, amp;
  Measure<Angle> northYaw, sourceYaw;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  public Drive(SwerveSubsystem keybotSwerve, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rAxis, Trigger slowModeMultiplier, Trigger chain, Trigger source, Trigger north, Trigger east,
      Trigger south, Trigger west, Trigger amp) {
    this.keybotSwerve = keybotSwerve;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rAxis = rAxis;
    this.slowModeMultiplier = slowModeMultiplier;
    this.source = source;
    this.north = north;
    this.east = east;
    this.south = south;
    this.west = west;
    this.chain = chain;
    this.amp = amp;

    isOpenLoop = true;

    addRequirements(this.keybotSwerve);
  }

  @Override
  public void initialize() {
    northYaw = Constants.fieldConstants.isRedAlliance() ? Units.Degrees.of(180) : Units.Degrees.of(0);
    sourceYaw = Units.Radians.of(Constants.fieldConstants.getFieldPositions().get()[2].getRotation().getZ());
    redAllianceMultiplier = Constants.fieldConstants.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    if (slowModeMultiplier.getAsBoolean()) {
      slowMultiplier = swervePreferences.slowModeMultiplier.getValue();
    } else {
      slowMultiplier = 1;
    }

    double translationMultiplier = slowMultiplier * redAllianceMultiplier
        * swervePreferences.driveSpeed.getValue();

    Measure<Velocity<Distance>> xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * translationMultiplier);
    Measure<Velocity<Distance>> yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * translationMultiplier);

    Measure<Velocity<Angle>> rVelocity = Units.RadiansPerSecond
        .of(swervePreferences.maxManualTurnSpeed.in(Units.DegreesPerSecond))
        .times(-rAxis.getAsDouble());
      
    keybotSwerve.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
        rVelocity.in(Units.RadiansPerSecond), isOpenLoop);

    }
    

  @Override
  public void end(boolean interrupted) {
    keybotSwerve.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}