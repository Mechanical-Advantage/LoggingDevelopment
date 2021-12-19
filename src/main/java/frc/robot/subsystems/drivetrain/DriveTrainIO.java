// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Drive subsystem hardware interface. */
public interface DriveTrainIO {
  /** The set of loggable inputs for the drive subsystem. */
  public static class DriveTrainInputs implements LoggableInputs {
    public double leftPositionRadians = 0.0;
    public double rightPositionRadians = 0.0;
    public double leftVelocityRadiansPerSecond = 0.0;
    public double rightVelocityRadiansPerSecond = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double gyroPositionRadians = 0.0;
    public double gyroVelocityRadiansPerSecond = 0.0;

    public void toLog(LogTable table) {
      table.put("LeftPositionRadians", leftPositionRadians);
      table.put("RightPositionRadians", rightPositionRadians);
      table.put("LeftVelocityRadiansPerSecond", leftVelocityRadiansPerSecond);
      table.put("RightVelocityRadiansPerSecond", rightVelocityRadiansPerSecond);
      table.put("LeftCurrentAmps", leftCurrentAmps);
      table.put("RightCurrentAmps", rightCurrentAmps);
      table.put("GyroPositionRadians", gyroPositionRadians);
      table.put("GyroVelocityRadiansPerSecond", gyroVelocityRadiansPerSecond);
    }

    public void fromLog(LogTable table) {
      leftPositionRadians = table.getDouble("LeftPositionRadians", leftPositionRadians);
      rightPositionRadians = table.getDouble("RightPositionRadians", rightPositionRadians);
      leftVelocityRadiansPerSecond = table.getDouble("LeftVelocityRadiansPerSecond", leftVelocityRadiansPerSecond);
      rightVelocityRadiansPerSecond = table.getDouble("RightVelocityRadiansPerSecond", rightVelocityRadiansPerSecond);
      leftCurrentAmps = table.getDouble("LeftCurrentAmps", leftPositionRadians);
      rightCurrentAmps = table.getDouble("RightCurrentAmps", rightPositionRadians);
      gyroPositionRadians = table.getDouble("GyroPositionRadians", gyroPositionRadians);
      gyroVelocityRadiansPerSecond = table.getDouble("GyroVelocityRadiansPerSecond", gyroVelocityRadiansPerSecond);
    }
  }

  /**
   * Updates the set of loggable inputs.
   * 
   * @param inputs The inputs to update.
   */
  public default void updateInputs(DriveTrainInputs inputs) {
  }

  /**
   * Set the output voltage (open loop control).
   * 
   * @param leftVoltage  Voltage for the left motors.
   * @param rightVoltage Voltage for the right motors.
   */
  public default void setOutputVolts(double leftVoltage, double rightVoltage) {
  }
}
