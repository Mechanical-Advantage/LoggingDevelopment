// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.core;

import java.util.Map;

/**
 * A set of values which can be logged and replayed (for example, the hardware
 * inputs for a subsystem). Data must be represented as a map with String keys
 * and values as one of the following types: integer, integer[], double,
 * double[], String, String[], boolean, boolean[], byte, byte[]
 */
public interface LoggableInputs {
  /**
   * Returns a Map representation of the data to log.
   */
  public Map<String, Object> toMap();

  /**
   * Updates data based on a provided Map representation.
   */
  public void fromMap(Map<String, Object> map);
}
