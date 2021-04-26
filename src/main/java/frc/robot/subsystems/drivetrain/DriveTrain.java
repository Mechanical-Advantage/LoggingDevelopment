// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.logging.core.Logger;

public class DriveTrain extends SubsystemBase {

  private static final double wheelRadiusMeters = Units.inchesToMeters(3.0);

  private final DriveTrainIO io;
  private final DriveTrainInputs inputs = new DriveTrainInputs();

  /** Creates a new DriveTrain. */
  public DriveTrain(DriveTrainIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("DriveTrain", inputs);
  }

  /**
   * Drive open loop with percent out.
   */
  public void drive(double leftPercent, double rightPercent) {
    Logger.getInstance().recordOutput("DriveTrain/LeftPercentOut", leftPercent);
    Logger.getInstance().recordOutput("DriveTrain/RightPercentOut", rightPercent);
    io.setOutputVolts(leftPercent * 12, rightPercent * 12);
  }

  /**
   * Returns the position of the left drive in meters.
   */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRadians * wheelRadiusMeters;
  }

  /**
   * Returns the position of the right drive in meters.
   */
  public double getRightPositionMeters() {
    return inputs.rightPositionRadians * wheelRadiusMeters;
  }
}
