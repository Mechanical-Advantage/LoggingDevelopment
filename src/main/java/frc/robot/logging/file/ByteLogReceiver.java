// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.file;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.logging.robot.LogRawDataReceiver;
import frc.robot.logging.shared.ByteEncoder;

/** Records log values to a custom binary format. */
public class ByteLogReceiver implements LogRawDataReceiver {

  private final String filename;

  FileOutputStream file;
  ByteEncoder encoder;

  public ByteLogReceiver(String filename) {
    this.filename = filename;
  }

  public void start(ByteEncoder encoder) {
    this.encoder = encoder;
    try {
      file = new FileOutputStream(filename);
    } catch (FileNotFoundException e) {
      DriverStation.reportError("Failed to open log file. Data will NOT be recorded.", true);
    }
  }

  public void end() {
    if (file != null) {
      try {
        file.close();
        file = null;
      } catch (IOException e) {
        DriverStation.reportError("Failed to close log file.", true);
      }
    }
  }

  public void processEntry() {
    if (file != null) {
      try {
        file.write(encoder.getOutput().array());
      } catch (IOException e) {
        DriverStation.reportError("Failed to write data to log file.", true);
      }
    }
  }
}