// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.core;

import java.util.HashMap;
import java.util.Map;

/**
 * A table of logged data in allowable types. Can reference another higher level
 * table.
 */
public class LogTable {
  private final double timestamp;
  private final String prefix;
  private final Map<String, Object> data;

  /**
   * Creates a new LogTable, to serve as the root table.
   */
  public LogTable(double timestamp) {
    this.timestamp = timestamp;
    prefix = "/";
    data = new HashMap<String, Object>();
  }

  /**
   * Creates a new LogTable, to reference a subtable.
   */
  private LogTable(double timestamp, String prefix, Map<String, Object> data) {
    this.timestamp = timestamp;
    this.prefix = prefix;
    this.data = data;
  }

  /**
   * Returns the timestamp of the table.
   */
  public double getTimestamp() {
    return timestamp;
  }

  /**
   * Creates a new LogTable for referencing a single subtable. Modifications to
   * the subtable will be reflected in the original object.
   * 
   * @param tableName The name of the subtable. Do not include a trailing slash.
   * @return The subtable object.
   */
  public LogTable getSubtable(String tableName) {
    return new LogTable(timestamp, prefix + tableName + "/", data);
  }

  /**
   * Returns all values from the table.
   * 
   * @param subtableOnly If true, include only values in the subtable (no prefix).
   *                     If false, include all values.
   * @return Map of the requested data.
   */
  public Map<String, Object> getAll(boolean subtableOnly) {
    if (subtableOnly) {
      Map<String, Object> result = new HashMap<String, Object>();
      for (Map.Entry<String, Object> field : data.entrySet()) {
        if (field.getKey().startsWith(prefix)) {
          result.put(field.getKey().substring(prefix.length()), field.getValue());
        }
      }
      return result;
    } else {
      return data;
    }
  }

  /** Internal method for retrieving value of given type. */
  private Object get(String key, Object defaultValue, LoggableType type) {
    String fullKey = prefix + key;
    if (data.containsKey(fullKey)) {
      Object value = data.get(fullKey);
      if (LoggableType.identify(value) == type) {
        return value;
      }
    }
    return defaultValue;
  }

  /** Writes a new Boolean value to the table. */
  public void put(String key, boolean value) {
    data.put(prefix + key, value);
  }

  /** Writes a new BooleanArray value to the table. */
  public void put(String key, boolean[] value) {
    data.put(prefix + key, value);
  }

  /** Writes a new Integer value to the table. */
  public void put(String key, int value) {
    data.put(prefix + key, value);
  }

  /** Writes a new IntegerArray value to the table. */
  public void put(String key, int[] value) {
    data.put(prefix + key, value);
  }

  /** Writes a new Double value to the table. */
  public void put(String key, double value) {
    data.put(prefix + key, value);
  }

  /** Writes a new DoubleArray value to the table. */
  public void put(String key, double[] value) {
    data.put(prefix + key, value);
  }

  /** Writes a new String value to the table. */
  public void put(String key, String value) {
    data.put(prefix + key, value);
  }

  /** Writes a new StringArray value to the table. */
  public void put(String key, String[] value) {
    data.put(prefix + key, value);
  }

  /** Writes a new Byte value to the table. */
  public void put(String key, byte value) {
    data.put(prefix + key, value);
  }

  /** Writes a new ByteArray value to the table. */
  public void put(String key, byte[] value) {
    data.put(prefix + key, value);
  }

  /** Reads a Boolean value from the table. */
  public boolean getBoolean(String key, boolean defaultValue) {
    return (boolean) get(key, defaultValue, LoggableType.Boolean);
  }

  /** Reads a BooleanArray value from the table. */
  public boolean[] getBooleanArray(String key, boolean[] defaultValue) {
    return (boolean[]) get(key, defaultValue, LoggableType.BooleanArray);
  }

  /** Reads a Integer value from the table. */
  public int getInteger(String key, int defaultValue) {
    return (int) get(key, defaultValue, LoggableType.Integer);
  }

  /** Reads a IntegerArray value from the table. */
  public int[] getIntegerArray(String key, int[] defaultValue) {
    return (int[]) get(key, defaultValue, LoggableType.IntegerArray);
  }

  /** Reads a Double value from the table. */
  public double getDouble(String key, double defaultValue) {
    return (double) get(key, defaultValue, LoggableType.Double);
  }

  /** Reads a DoubleArray value from the table. */
  public double[] getDoubleArray(String key, double[] defaultValue) {
    return (double[]) get(key, defaultValue, LoggableType.DoubleArray);
  }

  /** Reads a String value from the table. */
  public String getString(String key, String defaultValue) {
    return (String) get(key, defaultValue, LoggableType.String);
  }

  /** Reads a StringArray value from the table. */
  public String[] getStringArray(String key, String[] defaultValue) {
    return (String[]) get(key, defaultValue, LoggableType.StringArray);
  }

  /** Reads a Byte value from the table. */
  public byte getByte(String key, byte defaultValue) {
    return (byte) get(key, defaultValue, LoggableType.Byte);
  }

  /** Reads a ByteArray value from the table. */
  public byte[] getByteArray(String key, byte[] defaultValue) {
    return (byte[]) get(key, defaultValue, LoggableType.ByteArray);
  }

  /**
   * Represents all possible data types that can be logged.
   */
  public enum LoggableType {
    Boolean, BooleanArray, Integer, IntegerArray, Double, DoubleArray, String, StringArray, Byte, ByteArray;

    public static LoggableType identify(Object object) {
      if (object == null) {
        return null;
      }
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