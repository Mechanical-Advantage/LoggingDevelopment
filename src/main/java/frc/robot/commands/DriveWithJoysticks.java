// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class DriveWithJoysticks extends CommandBase {

  private static final double deadband = 0.1;
  private static final double maxSpeed = 0.5;

  private final DriveTrain driveTrain;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(DriveTrain driveTrain, DoubleSupplier leftY, DoubleSupplier rightX) {
    this.driveTrain = driveTrain;
    this.leftY = leftY;
    this.rightX = rightX;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private double processAxis(double input) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      double output = Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
      return output * Math.abs(output);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double baseSpeed = processAxis(leftY.getAsDouble() * -1);
    double turnSpeed = processAxis(rightX.getAsDouble());
    driveTrain.drivePercent((baseSpeed + turnSpeed) * maxSpeed, (baseSpeed - turnSpeed) * maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drivePercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
