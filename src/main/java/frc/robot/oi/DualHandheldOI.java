// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualHandheldOI implements HandheldOI {
  private final XboxController driverController;
  private final XboxController operatorController;

  public DualHandheldOI(int driverPort, int operatorPort) {
    driverController = new XboxController(driverPort);
    operatorController = new XboxController(operatorPort);
  }

  public double getLeftDriveX() {
    return driverController.getLeftX();
  }

  public double getLeftDriveY() {
    return driverController.getLeftY() * -1;
  }

  public double getRightDriveX() {
    return driverController.getRightX();
  }

  public double getRightDriveY() {
    return driverController.getRightY() * -1;
  }

  public Trigger getAutoAimButton() {
    return new Trigger(driverController::getLeftBumper);
  }

  public Trigger getIntakeButton() {
    return new Trigger(operatorController::getAButton);
  }

  public Trigger getShootButton() {
    return new Trigger(operatorController::getBButton);
  }
}
