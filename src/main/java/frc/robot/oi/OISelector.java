// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for selecting the appropriate OI implementations based on the
 * connected joysticks.
 */
public class OISelector {
  private static final String overrideJoystickName = "Generic HID Controller";
  private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

  private OISelector() {
  }

  /**
   * Returns whether the connected joysticks have changed since the last time this
   * method was called.
   */
  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port);
      if (!name.equals(lastJoystickNames[port])) {
        joysticksChanged = true;
        lastJoystickNames[port] = name;
      }
    }
    return joysticksChanged;
  }

  /**
   * Instantiates and returns an appropriate override OI object based on the
   * connected joysticks.
   */
  public static OverrideOI findOverrideOI() {
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.getJoystickName(port).equals(overrideJoystickName)) {
        return new OverrideOI(port);
      }
    }
    return new OverrideOI(); // No controller, return dummy OI
  }

  /**
   * Instantiates and returns an appropriate handheld OI object (driver and
   * operator) based on the connected joysticks.
   */
  public static HandheldOI findHandheldOI() {
    Integer driverPort = null;
    Integer operatorPort = null;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.getJoystickIsXbox(port)) {
        if (driverPort == null) {
          driverPort = port;
        } else if (operatorPort == null) {
          operatorPort = port;
        }
      }
    }

    if (driverPort == null) {
      return new HandheldOI() { // No controllers, use dummy OI
      };
    } else if (operatorPort == null) {
      return new SingleHandheldOI(driverPort); // No operator, use single controller
    } else {
      return new DualHandheldOI(driverPort, operatorPort); // Dual controller
    }
  }
}
