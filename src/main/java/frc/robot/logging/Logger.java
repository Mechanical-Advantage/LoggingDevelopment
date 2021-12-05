// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.logging.inputs.*;
import frc.robot.logging.io.ByteEncoder;
import frc.robot.logging.io.LogDataReceiver;
import frc.robot.logging.io.LogRawDataReceiver;
import frc.robot.logging.io.LogReplaySource;

/** Central class for recording and replaying log data. */
public class Logger {

  private static final boolean debugTiming = false;

  private static Logger instance;

  private boolean running = false;
  private final Timer printTimer = new Timer();
  private LogTable entry;
  private LogTable outputTable;
  private Map<String, String> metadata = new HashMap<>();
  private ByteEncoder encoder;

  private LogReplaySource replaySource;
  private List<LogDataReceiver> dataReceivers = new ArrayList<>();
  private List<LogRawDataReceiver> rawDataReceivers = new ArrayList<>();

  private Logger() {
  }

  public static Logger getInstance() {
    if (instance == null) {
      instance = new Logger();
    }
    return instance;
  }

  /**
   * Sets the source to use for replaying data. Use null to disable replay. This
   * method only work during setup before starting to log.
   */
  public void setReplaySource(LogReplaySource replaySource) {
    if (!running) {
      this.replaySource = replaySource;
    }
  }

  /**
   * Adds a new data receiver to process real or replayed data. This method only
   * work during setup before starting to log.
   */
  public void addDataReceiver(LogDataReceiver dataReceiver) {
    if (!running) {
      dataReceivers.add(dataReceiver);
    }
  }

  /**
   * Adds a new raw data receiver to process real or replayed data. This method
   * only work during setup before starting to log.
   */
  public void addDataReceiver(LogRawDataReceiver dataReceiver) {
    if (!running) {
      rawDataReceivers.add(dataReceiver);
    }
  }

  /**
   * Records a metadata value. This method only works during setup before starting
   * to log, then data will be recorded during the first cycle.
   * 
   * @param key   The name used to identify this metadata field.
   * @param value The value of the metadata field.
   */
  public void recordMetadata(String key, String value) {
    if (!running) {
      metadata.put(key, value);
    }
  }

  /**
   * Returns whether a replay source is currently being used.
   */
  public boolean hasReplaySource() {
    return replaySource != null;
  }

  /**
   * Starts running the logging system, including any data receivers or the replay
   * source.
   */
  public void start() {
    if (!running) {
      running = true;
      entry = null;

      if (rawDataReceivers.size() > 0) {
        encoder = new ByteEncoder();
      } else {
        encoder = null;
      }

      if (replaySource != null) {
        replaySource.start();
      }
      for (int i = 0; i < dataReceivers.size(); i++) {
        if (dataReceivers.get(i) != replaySource) {
          dataReceivers.get(i).start();
        }
      }
      for (int i = 0; i < rawDataReceivers.size(); i++) {
        if (rawDataReceivers.get(i) != replaySource) {
          rawDataReceivers.get(i).start(encoder);
        }
      }

      printTimer.reset();
      printTimer.start();
      periodic();

      // Record metadata
      LogTable metadataTable = entry.getSubtable(replaySource == null ? "RealMetadata" : "ReplayMetadata");
      for (Map.Entry<String, String> item : metadata.entrySet()) {
        metadataTable.put(item.getKey(), item.getValue());
      }
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
      for (int i = 0; i < rawDataReceivers.size(); i++) {
        if (rawDataReceivers.get(i) != replaySource) {
          rawDataReceivers.get(i).end();
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
        if (rawDataReceivers.size() > 0) {
          encoder.encodeTable(entry);
        }
        for (int i = 0; i < rawDataReceivers.size(); i++) {
          rawDataReceivers.get(i).processEntry();
        }
      }

      // Get next entry
      if (replaySource == null) {
        entry = new LogTable(Timer.getFPGATimestamp());
        outputTable = entry.getSubtable("RealOutputs");
      } else {
        entry = replaySource.getEntry();
        if (entry == null) {
          end();
          System.exit(0);
        }
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
