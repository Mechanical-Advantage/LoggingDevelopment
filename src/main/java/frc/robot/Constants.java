// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean tuningMode = false;
  public static final boolean driveTrainOpenLoop = true;
  private static final Robot robot = Robot.KITBOT;
  private static final Robot defaultRobot = Robot.KITBOT;
  public static final double loopPeriodSecs = 0.02;

  public static Robot getRobot() {
    if (RobotBase.isReal()) {
      if (robot == Robot.SIMBOT || robot == Robot.ROMI) { // Invalid robot selected
        return defaultRobot;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case KITBOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT:
      case ROMI:
        return Mode.SIM;
      default:
        return Mode.REAL;
    }
  }

  public static enum Robot {
    KITBOT, SIMBOT, ROMI
  }

  public static enum Mode {
    REAL, REPLAY, SIM
  }
}
