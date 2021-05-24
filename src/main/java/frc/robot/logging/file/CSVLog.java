// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.file;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

import frc.robot.logging.core.LogDataReceiver;
import frc.robot.logging.core.LogTable;
import frc.robot.logging.core.LogTable.LoggableType;

/** Records log values to a CSV file. */
public class CSVLog implements LogDataReceiver {

  private static final String filename = "/media/sda1/robotlog.csv";
  FileWriter csvWriter;

  public void start() {
    try {
      // Create log file
      File csv = new File(filename);
      if (csv.createNewFile()) {
        System.out.println("Successfully created log file.");
      } else {
        System.out.println("Log file already exists, will be reused.");
      }

      // Open log file
      csvWriter = new FileWriter(filename);
    } catch (IOException e) {
      System.out.println("Could not open log file.");
      e.printStackTrace();
    }
  }

  public void end() {
    try {
      csvWriter.close();
    } catch (IOException e) {
      System.out.println("Could not close log file.");
      e.printStackTrace();
    }
  }

  public void putEntry(LogTable entry) {
    String timestampString = Double.toString(entry.getTimestamp());
    try {
      for (Map.Entry<String, Object> field : entry.getAll(false).entrySet()) {
        LoggableType type = LoggableType.identify(field.getValue());
        csvWriter.write(timestampString + "," + field.getKey() + "," + type.name());
        switch (type) {
          case Boolean:
            csvWriter.write((boolean) field.getValue() == true ? ",true" : ",false");
            break;
          case BooleanArray:
            boolean[] booleanArray = (boolean[]) field.getValue();
            for (int i = 0; i < booleanArray.length; i++) {
              csvWriter.write(booleanArray[i] == true ? ",true" : ",false");
            }
            break;
          case Integer:
            csvWriter.write("," + Integer.toString((int) field.getValue()));
            break;
          case IntegerArray:
            int[] intArray = (int[]) field.getValue();
            for (int i = 0; i < intArray.length; i++) {
              csvWriter.write("," + Integer.toString(intArray[i]));
            }
            break;
          case Double:
            csvWriter.write("," + Double.toString((double) field.getValue()));
            break;
          case DoubleArray:
            double[] doubleArray = (double[]) field.getValue();
            for (int i = 0; i < doubleArray.length; i++) {
              csvWriter.write("," + Double.toString(doubleArray[i]));
            }
            break;
          case String:
            csvWriter.write("," + (String) field.getValue());
            break;
          case StringArray:
            String[] stringArray = (String[]) field.getValue();
            for (int i = 0; i < stringArray.length; i++) {
              csvWriter.write("," + stringArray[i]);
            }
            break;
          case Byte:
            csvWriter.write("," + Byte.toString((byte) field.getValue()));
            break;
          case ByteArray:
            byte[] byteArray = (byte[]) field.getValue();
            for (int i = 0; i < byteArray.length; i++) {
              csvWriter.write("," + Byte.toString(byteArray[i]));
            }
            break;
        }
        csvWriter.write("\n");
      }
    } catch (IOException e) {
      System.out.println("Could not write to log file.");
      e.printStackTrace();
    }
  }
}
