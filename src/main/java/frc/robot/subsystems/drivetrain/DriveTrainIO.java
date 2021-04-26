// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

/** Drive subsystem hardware interface. */
public interface DriveTrainIO {

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
