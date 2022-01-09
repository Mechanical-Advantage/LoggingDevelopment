// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements HandheldOI {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  public double getLeftDriveX() {
    return controller.getLeftX();
  }

  public double getLeftDriveY() {
    return controller.getLeftY() * -1;
  }

  public double getRightDriveX() {
    return controller.getRightX();
  }

  public double getRightDriveY() {
    return controller.getRightY() * -1;
  }

  public Trigger getAutoAimButton() {
    return new Trigger(controller::getLeftBumper);
  }

  public Trigger getIntakeButton() {
    return new Trigger(controller::getAButton);
  }

  public Trigger getShootButton() {
    return new Trigger(controller::getBButton);
  }
}
