// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.core;

import java.util.HashMap;
import java.util.Map;

/**
 * A set of logged data and timestamp.
 */
public class LogEntry {
  public double timestamp;
  private Map<String, Object> data = new HashMap<String, Object>();

  /**
   * Adds a single field to the log entry. This is the full key, so it should
   * include a leading slash.
   */
  public void addField(String key, Object value) {
    if (LoggableType.identify(value) != null) {
      data.put(key, value);
    }
  }

  /**
   * Gets a single field from the log entry. This is the full key, so it should
   * include a leading slash.
   */
  public Object getField(String key) {
    return data.get(key);
  }

  /**
   * Adds a table to the log entry. The table key and field keys should not start
   * with slashes.
   */
  public void addTable(String tableKey, Map<String, Object> values) {
    for (Map.Entry<String, Object> field : values.entrySet()) {
      addField("/" + tableKey + "/" + field.getKey(), field.getValue());
    }
  }

  /**
   * Gets a table from the log entry. The table key should not start with a slash.
   */
  public Map<String, Object> getTable(String tableKey) {
    Map<String, Object> tableData = new HashMap<String, Object>();
    for (Map.Entry<String, Object> field : data.entrySet()) {
      if (field.getKey().startsWith("/" + tableKey + "/")) {
        tableData.put(field.getKey().substring(tableKey.length() + 2), field.getValue());
      }
    }
    return tableData;
  }

  /**
   * Returns all of the data currently stored in the entry.
   */
  public Map<String, Object> getAll() {
    return data;
  }

  /**
   * Returns a map of the field changes beween the old entry and this entry;
   */
  public Map<String, Object> getChanges(LogEntry oldEntry) {
    Map<String, Object> oldData = oldEntry.getAll();
    Map<String, Object> changes = new HashMap<String, Object>();

    for (Map.Entry<String, Object> newField : data.entrySet()) {
      if (oldData.containsKey(newField.getKey())) {
        if (oldData.get(newField.getKey()) == newField.getValue()) {
          continue;
        }
      }
      changes.put(newField.getKey(), newField.getValue());
    }
    return changes;
  }

  /**
   * Represents all possible data types that can be logged.
   */
  public enum LoggableType {
    Boolean, BooleanArray, Integer, IntegerArray, Double, DoubleArray, String, StringArray, Byte, ByteArray;

    public static LoggableType identify(Object object) {
      if (object.getClass().isArray()) {
        if (object.getClass().getComponentType() == boolean.class
            || object.getClass().getComponentType() == Boolean.class) {
          return BooleanArray;
        }
        if (object.getClass().getComponentType() == int.class
            || object.getClass().getComponentType() == Integer.class) {
          return IntegerArray;
        }
        if (object.getClass().getComponentType() == double.class
            || object.getClass().getComponentType() == Double.class) {
          return DoubleArray;
        }
        if (object.getClass().getComponentType() == String.class) {
          return StringArray;
        }
        if (object.getClass().getComponentType() == byte.class || object.getClass().getComponentType() == Byte.class) {
          return ByteArray;
        }
      } else {
        if (object.getClass() == Boolean.class) {
          return Boolean;
        }
        if (object.getClass() == Integer.class) {
          return Integer;
        }
        if (object.getClass() == Double.class) {
          return Double;
        }
        if (object.getClass() == String.class) {
          return String;
        }
        if (object.getClass() == Byte.class) {
          return Byte;
        }
      }
      return null;
    }
  }
}