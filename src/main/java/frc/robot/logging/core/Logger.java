// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.core;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.logging.inputs.*;

/** Central class for recording and replaying log data. */
public class Logger {

  private static final boolean debugTiming = false;

  private static Logger instance;

  private boolean running = false;
  private LogTable entry;
  private LogTable outputTable;
  private LogReplaySource replaySource;
  private List<LogDataReceiver> dataReceivers = new ArrayList<>();
  private final Timer printTimer = new Timer();

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
      printTimer.reset();
      printTimer.start();
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
      double periodicStart = Timer.getFPGATimestamp();

      // Send data to receivers
      if (entry != null) {
        for (int i = 0; i < dataReceivers.size(); i++) {
          dataReceivers.get(i).putEntry(entry);
        }
      }

      // Get next entry
      if (replaySource == null) {
        entry = new LogTable(Timer.getFPGATimestamp());
        outputTable = entry.getSubtable("RealOutputs");
      } else {
        entry = replaySource.getEntry();
        outputTable = entry.getSubtable("ReplayOutputs");
      }

      // Update default inputs
      double driverStationStart = Timer.getFPGATimestamp();
      LoggedDriverStation.getInstance().periodic();
      double systemStatsStart = Timer.getFPGATimestamp();
      processInputs("SystemStats", LoggedSystemStats.getInstance());
      double networkTablesStart = Timer.getFPGATimestamp();
      processInputs("NetworkTables", LoggedNetworkTables.getInstance());
      double periodicEnd = Timer.getFPGATimestamp();

      // Print timing data
      if (printTimer.advanceIfElapsed(0.5) && debugTiming) {
        String updateLength = Double.toString((double) Math.round((driverStationStart - periodicStart) * 100000) / 100);
        String driverStationLength = Double
            .toString((double) Math.round((systemStatsStart - driverStationStart) * 100000) / 100);
        String systemStatsLength = Double
            .toString((double) Math.round((networkTablesStart - systemStatsStart) * 100000) / 100);
        String networkTablesLength = Double
            .toString((double) Math.round((periodicEnd - networkTablesStart) * 100000) / 100);
        System.out.println("U=" + updateLength + ", DS=" + driverStationLength + ", SS=" + systemStatsLength + ", NT="
            + networkTablesLength);
      }
    }
  }

  /**
   * Returns the current FPGA timestamp or replayed time. Use this or
   * LoggedTimer.getFPGATimestamp() instead of Timer.getFPGATimestamp().
   */
  public double getTimestamp() {
    return entry.getTimestamp();
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
        inputs.toLog(entry.getSubtable(key));
      } else {
        inputs.fromLog(entry.getSubtable(key));
      }
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, Boolean value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, boolean[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, Integer value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, int[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, Double value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, double[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, String value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, String[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, byte value) {
    if (running) {
      outputTable.put(key, value);
    }
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public void recordOutput(String key, byte[] value) {
    if (running) {
      outputTable.put(key, value);
    }
  }
}
