// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.Map;

import frc.robot.logging.core.LoggableInputs;

/** The set of loggable inputs for the drive subsystem. */
public class DriveTrainInputs implements LoggableInputs {
  public double leftPositionRadians = 0.0;
  public double rightPositionRadians = 0.0;
  public double leftCurrentAmps = 0.0;
  public double rightCurrentAmps = 0.0;
  public double gyroAngleDegrees = 0.0;

  public Map<String, Object> toMap() {
    return Map.of("LeftPositionRadians", leftPositionRadians, "RightPositionRadians", rightPositionRadians,
        "LeftCurrentAmps", leftCurrentAmps, "RightCurrentAmps", rightCurrentAmps, "GyroAngleDegrees", gyroAngleDegrees);
  }

  public void fromMap(Map<String, Object> map) {
    leftPositionRadians = (double) map.getOrDefault("LeftPositionRadians", leftPositionRadians);
    rightPositionRadians = (double) map.getOrDefault("RightPositionRadians", rightPositionRadians);
    leftCurrentAmps = (double) map.getOrDefault("LeftCurrentAmps", leftPositionRadians);
    rightCurrentAmps = (double) map.getOrDefault("RightCurrentAmps", rightPositionRadians);
    gyroAngleDegrees = (double) map.getOrDefault("GyroAngleDegrees", gyroAngleDegrees);
  }
}
