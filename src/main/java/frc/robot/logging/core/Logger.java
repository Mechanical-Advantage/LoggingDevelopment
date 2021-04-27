// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.core;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.logging.inputs.*;

/** Central class for recording and replaying log data. */
public class Logger {

  private static Logger instance;

  private boolean running = false;
  private LogEntry entry;
  private LogReplaySource replaySource;
  private List<LogDataReceiver> dataReceivers = new ArrayList<>();

  private Logger() {
  }

  public static Logger getInstance() {
    if (instance == null) {
      instance = new Logger();
    }
    return instance;
  }

  /**
   * Sets the source to use for replaying data. Use null to disable replay.
   */
  public void setReplaySource(LogReplaySource replaySource) {
    this.replaySource = replaySource;
  }

  /**
   * Adds a new data receiver to process real or replayed data.
   */
  public void addDataReceiver(LogDataReceiver dataReceiver) {
    dataReceivers.add(dataReceiver);
  }

  /**
   * Starts running the logging system, including any data receivers or the replay
   * source.
   */
  public void start() {
    if (!running) {
      running = true;
      if (replaySource != null) {
        replaySource.start();
      }
      for (int i = 0; i < dataReceivers.size(); i++) {
        if (dataReceivers.get(i) != replaySource) {
          dataReceivers.get(i).start();
        }
      }
      periodic();
    }
  }

  /**
   * Ends the logging system, including any data receivers or the replay source.
   */
  public void end() {
    if (running) {
      running = false;
      if (replaySource != null) {
        replaySource.end();
      }
      for (int i = 0; i < dataReceivers.size(); i++) {
        if (dataReceivers.get(i) != replaySource) {
          dataReceivers.get(i).end();
        }
      }
    }
  }

  /**
   * Periodic method to be called before robotInit and each loop cycle. Updates
   * timestamp and globally logged data.
   */
  public void periodic() {
    if (running) {
      // Send data to receivers
      if (entry != null) {
        for (int i = 0; i < dataReceivers.size(); i++) {
          dataReceivers.get(i).putEntry(entry);
        }
      }

      // Get next entry
      if (replaySource == null) {
        entry = new LogEntry();
        entry.timestamp = Timer.getFPGATimestamp();
      } else {
        entry = replaySource.getEntry();
      }

      // Update default inputs
      LoggedDriverStation.getInstance().periodic();
      processInputs("SystemStats", LoggedSystemStats.getInstance());
      processInputs("NetworkTables", LoggedNetworkTables.getInstance());
    }
  }

  /**
   * Processes a set of inputs, logging them on the real robot or updating them in
   * the simulator. This should be called every loop cycle after updating the
   * inputs from the hardware (if applicable).
   * 
   * @param key    The name used to identify this set of inputs.
   * @param inputs The inputs to log or update.
   */
  public void processInputs(String key, LoggableInputs inputs) {
    if (running) {
      if (replaySource == null) {
        entry.addTable(key, inputs.toMap());
      } else {
        try {
          inputs.fromMap(entry.getTable(key));
        } catch (ClassCastException e) {
          DriverStation.reportError("Found log data that does not match expected field type.", false);
        }
      }
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field in one of the accepted types.
   */
  public void recordOutput(String key, Object value) {
    if (running) {
      entry.addField((replaySource == null ? "/RealOutputs/" : "/ReplayOutputs/") + key, value);
    }
  }

  /**
   * Returns the current FPGA timestamp or replayed time. Use this or
   * LoggedTimer.getFPGATimestamp() instead of Timer.getFPGATimestamp().
   */
  public double getTimestamp() {
    return entry.timestamp;
  }
}
