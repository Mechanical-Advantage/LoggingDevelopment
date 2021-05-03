// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.inputs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.logging.core.LoggableInputs;
import frc.robot.logging.core.LogEntry.LoggableType;
import edu.wpi.first.networktables.NetworkTable;

/**
 * Manages logging and replaying the contents of NetworkTables.
 */
public class LoggedNetworkTables implements LoggableInputs {

  private static LoggedNetworkTables instance;
  private static NetworkTableInstance networkTables = NetworkTableInstance.getDefault();

  private List<String> tables = new ArrayList<>();

  private LoggedNetworkTables() {
    tables.add("/SmartDashboard");
  }

  public static LoggedNetworkTables getInstance() {
    if (instance == null) {
      instance = new LoggedNetworkTables();
    }
    return instance;
  }

  /**
   * Adds a new table to log and replay ("/SmartDashboard" is included by
   * default). This should be called for any tables used in user code.
   * 
   * @param prefix The name of the table to add
   */
  public void addTable(String prefix) {
    tables.add(prefix);
  }

  public Map<String, Object> toMap() {
    Map<String, Object> map = new HashMap<String, Object>();

    for (int tableId = 0; tableId < tables.size(); tableId++) {
      NetworkTableEntry[] entries = networkTables.getEntries(tables.get(tableId), 0);
      for (int entryId = 0; entryId < entries.length; entryId++) {
        NetworkTableEntry entry = entries[entryId];
        String key = entry.getName().substring(1);
        switch (entry.getType()) {
          case kBoolean:
            map.put(key, entry.getBoolean(false));
            break;
          case kBooleanArray:
            map.put(key, entry.getBooleanArray(new boolean[0]));
            break;
          case kDouble:
            map.put(key, entry.getDouble(0.0));
            break;
          case kDoubleArray:
            map.put(key, entry.getDoubleArray(new double[0]));
            break;
          case kString:
            map.put(key, entry.getString(""));
            break;
          case kStringArray:
            map.put(key, entry.getStringArray(new String[0]));
            break;
          case kRaw:
            map.put(key, entry.getRaw(new byte[0]));
          default:
            break;
        }
      }
    }

    return map;
  }

  public void fromMap(Map<String, Object> map) {
    NetworkTable table = networkTables.getTable("/");

    for (Map.Entry<String, Object> mapEntry : map.entrySet()) {
      NetworkTableEntry tableEntry = table.getEntry(mapEntry.getKey());

      switch (LoggableType.identify(mapEntry.getValue())) {
        case Boolean:
          tableEntry.setBoolean((boolean) mapEntry.getValue());
          break;
        case BooleanArray:
          tableEntry.setBooleanArray((boolean[]) mapEntry.getValue());
          break;
        case Integer:
          tableEntry.setDouble((int) mapEntry.getValue());
          break;
        case IntegerArray:
          tableEntry.setDoubleArray(Arrays.stream((int[]) mapEntry.getValue()).asDoubleStream().toArray());
          break;
        case Double:
          tableEntry.setDouble((double) mapEntry.getValue());
          break;
        case DoubleArray:
          tableEntry.setDoubleArray((double[]) mapEntry.getValue());
          break;
        case String:
          tableEntry.setString((String) mapEntry.getValue());
          break;
        case StringArray:
          tableEntry.setStringArray((String[]) mapEntry.getValue());
          break;
        case ByteArray:
          tableEntry.setRaw((byte[]) mapEntry.getValue());
          break;
        default:
          break;
      }
    }
  }
}
