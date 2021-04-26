// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.inputs;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.logging.core.LoggableInputs;

/**
 * Manages logging general system data. This is NOT replayed to the simulator.
 */
public class LoggedSystemStats implements LoggableInputs {

  private static LoggedSystemStats instance;
  private static final PowerDistributionPanel pdp = new PowerDistributionPanel();

  private LoggedSystemStats() {
  }

  public static LoggedSystemStats getInstance() {
    if (instance == null) {
      instance = new LoggedSystemStats();
    }
    return instance;
  }

  public Map<String, Object> toMap() {
    Map<String, Object> map = new HashMap<String, Object>();
    map.put("BatteryVoltage", RobotController.getBatteryVoltage());
    map.put("BrownedOut", RobotController.isBrownedOut());
    map.put("CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);
    double[] pdpCurrents = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int channel = 0; channel < 16; channel++) {
      pdpCurrents[channel] = pdp.getCurrent(channel);
    }
    map.put("PDPCurrents", pdpCurrents);
    return map;
  }

  public void fromMap(Map<String, Object> map) {
    // Ignore replayed inputs
  }
}
