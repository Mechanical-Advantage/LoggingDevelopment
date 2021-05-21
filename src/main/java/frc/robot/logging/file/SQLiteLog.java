// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.file;

import java.io.File;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import org.tmatesoft.sqljet.core.SqlJetException;
import org.tmatesoft.sqljet.core.SqlJetTransactionMode;
import org.tmatesoft.sqljet.core.table.ISqlJetTable;
import org.tmatesoft.sqljet.core.table.SqlJetDb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.logging.core.LogDataReceiver;
import frc.robot.logging.core.LogTable;
import frc.robot.logging.core.LogTable.LoggableType;

/** Records log values to a SQLite database. */
public class SQLiteLog implements LogDataReceiver {

  private static final String filename = "/media/sda1/robotlog.db";
  private static final double writeFrequencySeconds = 2.0;

  private File dbFile = new File(filename);
  private SqlJetDb db;
  private ISqlJetTable table;

  private final BlockingQueue<LogTable> queue = new LinkedBlockingQueue<LogTable>();
  private SQLiteWriter writerThread;

  private boolean dbReady() {
    if (db == null) {
      return false;
    }
    return db.isOpen();
  }

  public void start() {
    // Create DB file
    dbFile.delete();
    try {
      db = SqlJetDb.open(dbFile, true);
      db.beginTransaction(SqlJetTransactionMode.WRITE);
      db.createTable(
          "CREATE TABLE records (timestamp INTEGER NOT NULL, key TEXT NOT NULL, type INTEGER NOT NULL, value BLOB)");
      db.commit();
      table = db.getTable("records");
    } catch (SqlJetException e) {
      DriverStation.reportError("Failed to prepare log file. Data will NOT be recorded.", true);
    }

    // Start writer thread
    writerThread = new SQLiteWriter();
    writerThread.start();
  }

  public void end() {
    if (writerThread != null) {
      writerThread.interrupt();
      writerThread.writeAll();
    }

    if (dbReady()) {
      try {
        db.close();
      } catch (SqlJetException e) {
        DriverStation.reportError("Failed to close log file.", true);
      }
    }
  }

  public void putEntry(LogTable entry) {
    try {
      queue.put(entry);
    } catch (InterruptedException e) {
    }
  }

  private class SQLiteWriter extends Thread {
    private LogTable lastEntry = new LogTable(0.0);

    @Override
    public void run() {
      try {
        while (true) {
          Thread.sleep((long) (writeFrequencySeconds * 1000));
          double startTime = Timer.getFPGATimestamp();
          writeAll();
          System.out
              .println("Wrote to DB in " + Double.toString((Timer.getFPGATimestamp() - startTime) * 1000) + " ms");
        }
      } catch (InterruptedException e) {
        DriverStation.reportWarning("SQLite writer thread interrupted.", true);
        if (dbReady()) {
          try {
            if (db.isInTransaction()) {
              db.commit();
            }
          } catch (SqlJetException e1) {
            DriverStation.reportWarning("Failed to write data to log file on interrupt.", true);
          }
        }
      }
    }

    public void writeAll() {
      if (dbReady()) {
        try {
          db.beginTransaction(SqlJetTransactionMode.WRITE);
          while (true) {
            LogTable entry = queue.poll();
            if (entry == null) {
              break;
            }

            Map<String, Object> newMap = entry.getAll(false);
            Map<String, Object> oldMap = lastEntry.getAll(false);

            // Write new data
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
              Object output;
              switch (newType) {
                case Boolean:
                case Integer:
                case Double:
                case String:
                case Byte:
                  output = newValue;
                  break;
                case BooleanArray:
                  output = "";
                  boolean[] booleanArray = (boolean[]) newValue;
                  for (int i = 0; i < booleanArray.length; i++) {
                    output += (i == 0 ? "" : ",") + (booleanArray[i] == true ? "1" : "0");
                  }
                  break;
                case IntegerArray:
                  output = "";
                  int[] intArray = (int[]) newValue;
                  for (int i = 0; i < intArray.length; i++) {
                    output += (i == 0 ? "" : ",") + Integer.toString(intArray[i]);
                  }
                  break;
                case DoubleArray:
                  output = "";
                  double[] doubleArray = (double[]) newValue;
                  for (int i = 0; i < doubleArray.length; i++) {
                    output += (i == 0 ? "" : ",") + Double.toString(doubleArray[i]);
                  }
                  break;
                case StringArray:
                  output = "";
                  String[] stringArray = (String[]) newValue;
                  for (int i = 0; i < stringArray.length; i++) {
                    output += (i == 0 ? "" : ",") + stringArray[i];
                  }
                  break;
                case ByteArray:
                  output = "";
                  byte[] byteArray = (byte[]) newValue;
                  for (int i = 0; i < byteArray.length; i++) {
                    output += (i == 0 ? "" : ",") + Byte.toString(byteArray[i]);
                  }
                  break;
                default:
                  output = null;
              }
              table.insert(entry.getTimestamp(), field.getKey(), newType.ordinal(), output);
            }

            // Find removed fields
            for (Map.Entry<String, Object> field : oldMap.entrySet()) {
              if (!newMap.containsKey(field.getKey())) {
                table.insert(entry.getTimestamp(), field.getKey(), LoggableType.identify(field.getValue()).ordinal(),
                    null);
              }
            }

            lastEntry = entry;
          }
          db.commit();
        } catch (SqlJetException e) {
          DriverStation.reportError("Failed to write data to log file.", true);
        }
      }
    }
  }

  public static void main(String... args) {
    SQLiteLog receiver = new SQLiteLog();
    receiver.start();

    LogTable testEntry1 = new LogTable(0.02);
    testEntry1.put("TestData/AInt", 6328);
    testEntry1.put("TestData/BDouble", 3.14159);
    testEntry1.put("TestData/CByteArray", new byte[] { 1, 2, 3, 4 });
    testEntry1.put("TestData/DBoolean", false);
    receiver.putEntry(testEntry1);

    LogTable testEntry2 = new LogTable(0.04);
    testEntry2.put("TestData/AInt", 254);
    testEntry2.put("TestData/BDouble", 3.14159);
    testEntry2.put("TestData/CByteArray", new byte[] { 1, 2, 3, 4 });
    receiver.putEntry(testEntry2);

    receiver.end();
  }
}