// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.inputs;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.logging.shared.LogTable;
import frc.robot.logging.robot.LoggableInputs;

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

  public void toLog(LogTable table) {
    table.put("BatteryVoltage", RobotController.getBatteryVoltage());
    table.put("BrownedOut", RobotController.isBrownedOut());
    table.put("CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);
    double[] pdpCurrents = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int channel = 0; channel < 16; channel++) {
      pdpCurrents[channel] = pdp.getCurrent(channel);
    }
    table.put("PDPCurrents", pdpCurrents);
  }

  public void fromLog(LogTable table) {
    // Ignore replayed inputs
  }
}
