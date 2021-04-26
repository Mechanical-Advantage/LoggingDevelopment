// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.core;

/**
 * Provides a stream of log entries to be fed back to the robot code during
 * simulation.
 */
public interface LogReplaySource {

  /**
   * Called before the logging system begins reporting data. This should be used
   * to connect to files, find network devices, start threads, etc.
   */
  public void start();

  /**
   * Called when the code shuts down cleanly. Note that this will NOT be called
   * when the robot is powered off.
   */
  public void end();

  /**
   * Called every loop cycle to get the next set of data. Fields from previous
   * cycles will NOT be preserved.
   */
  public LogEntry getEntry();
}
