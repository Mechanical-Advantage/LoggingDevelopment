// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** The set of loggable inputs for the drive subsystem. */
public class DriveTrainInputs implements LoggableInputs {
  public double leftPositionRadians = 0.0;
  public double rightPositionRadians = 0.0;
  public double leftCurrentAmps = 0.0;
  public double rightCurrentAmps = 0.0;
  public double gyroAngleDegrees = 0.0;

  public void toLog(LogTable table) {
    table.put("LeftPositionRadians", leftPositionRadians);
    table.put("RightPositionRadians", rightPositionRadians);
    table.put("LeftCurrentAmps", leftCurrentAmps);
    table.put("RightCurrentAmps", rightCurrentAmps);
    table.put("GyroAngleDegrees", gyroAngleDegrees);
  }

  public void fromLog(LogTable table) {
    leftPositionRadians = table.getDouble("LeftPositionRadians", leftPositionRadians);
    rightPositionRadians = table.getDouble("RightPositionRadians", rightPositionRadians);
    leftCurrentAmps = table.getDouble("LeftCurrentAmps", leftPositionRadians);
    rightCurrentAmps = table.getDouble("RightCurrentAmps", rightPositionRadians);
    gyroAngleDegrees = table.getDouble("GyroAngleDegrees", gyroAngleDegrees);
  }
}
