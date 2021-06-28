// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.robot;

import frc.robot.logging.shared.ByteEncoder;

/**
 * Receives entries (encoded as bytes) from the logging system during real
 * operation or simulation.
 */
public interface LogRawDataReceiver {

  /**
   * Called before the logging system begins reporting data. This should be used
   * to connect to files, find network devices, start threads, etc. The encoder
   * object provided here will be updated as records change.
   */
  public default void start(ByteEncoder encoder) {
  };

  /**
   * Called when the code shuts down cleanly. Note that this will NOT be called
   * when the robot is powered off.
   */
  public default void end() {
  };

  /**
   * Called every loop cycle when a new entry is complete. Call "getOutput()" on
   * the encoder to retrieve the encoded values. The data can be processed
   * immediately or queued for later.
   */
  public void processEntry();
}
