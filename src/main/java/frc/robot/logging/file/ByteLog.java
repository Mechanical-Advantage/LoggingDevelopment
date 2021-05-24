// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.file;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.logging.core.LogDataReceiver;
import frc.robot.logging.core.LogTable;
import frc.robot.logging.core.LogTable.LoggableType;

/** Records log values to a custom binary format. */
public class ByteLog implements LogDataReceiver {

  private static final String filename = "/media/sda1/robotlog.rlog";

  FileOutputStream file;
  LogTable lastEntry;
  Map<String, Short> keyIDs;
  short nextKeyID;

  public void start() {
    lastEntry = new LogTable(0.0);
    keyIDs = new HashMap<>();
    nextKeyID = 0;
    try {
      file = new FileOutputStream(filename);
    } catch (FileNotFoundException e) {
      DriverStation.reportError("Failed to open log file. Data will NOT be recorded.", true);
    }
  }

  public void end() {
    try {
      file.close();
    } catch (IOException e) {
      DriverStation.reportError("Failed to close log file.", true);
    }
  }

  private void writeField(String key, Object value) {
    try {
      if (!keyIDs.containsKey(key)) {
        keyIDs.put(key, nextKeyID);
        file.write(ByteEncoder.encodeKey(nextKeyID, key).array());
        nextKeyID++;
      }
      file.write(ByteEncoder.encodeValue(keyIDs.get(key), value).array());
    } catch (IOException e) {
      DriverStation.reportError("Failed to write data to log file.", true);
    }
  }

  public void putEntry(LogTable entry) {
    try {
      Map<String, Object> newMap = entry.getAll(false);
      Map<String, Object> oldMap = lastEntry.getAll(false);

      // Record timestamp
      file.write(ByteEncoder.encodeTimestamp(entry.getTimestamp()).array());

      for (Map.Entry<String, Object> field : newMap.entrySet()) {
        // Check if field has changed
        Object newValue = field.getValue();
        LoggableType newType = LoggableType.identify(newValue);
        boolean fieldChanged = true;
        if (oldMap.containsKey(field.getKey())) {
          Object oldValue = oldMap.get(field.getKey());
          LoggableType oldType = LoggableType.identify(oldValue);
          if (newType == oldType) {
            switch (newType) {
              case Boolean:
              case Integer:
              case Double:
              case String:
              case Byte:
                if (newValue.equals(oldValue)) {
                  fieldChanged = false;
                }
                break;
              case BooleanArray:
                if (Arrays.equals((boolean[]) newValue, (boolean[]) oldValue)) {
                  fieldChanged = false;
                }
                break;
              case IntegerArray:
                if (Arrays.equals((int[]) newValue, (int[]) oldValue)) {
                  fieldChanged = false;
                }
                break;
              case DoubleArray:
                if (Arrays.equals((double[]) newValue, (double[]) oldValue)) {
                  fieldChanged = false;
                }
                break;
              case StringArray:
                if (Arrays.equals((String[]) newValue, (String[]) oldValue)) {
                  fieldChanged = false;
                }
                break;
              case ByteArray:
                if (Arrays.equals((byte[]) newValue, (byte[]) oldValue)) {
                  fieldChanged = false;
                }
                break;
            }
          }
        }
        if (!fieldChanged) {
          continue;
        }

        // Write new data
        writeField(field.getKey(), newValue);
      }

      // Find removed fields
      for (Map.Entry<String, Object> field : oldMap.entrySet()) {
        if (!newMap.containsKey(field.getKey())) {
          writeField(field.getKey(), null);
        }
      }

      lastEntry = entry;
    } catch (IOException e) {
      DriverStation.reportError("Failed to write data to log file.", true);
    }
  }

  public static void main(String... args) {
    ByteLog log = new ByteLog();
    log.start();

    LogTable table1 = new LogTable(1.0);
    table1.put("TestValue", Math.random());
    log.putEntry(table1);

    LogTable table2 = new LogTable(2.0);
    // table1.put("TestValue", Math.random());
    log.putEntry(table2);

    log.end();

    // for (byte b : result) {
    // System.out.format("0x%x ", b);
    // }
  }
}