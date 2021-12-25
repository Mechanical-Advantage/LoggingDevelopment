// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.util.RomiGyro;

/** Drive subsystem hardware interface for the Romi. */
public class DriveTrainIORomi implements DriveTrainIO {

  private static final double radiansPerTick = (2.0 * Math.PI) / 1440.0;
  private static final boolean reverseOutputLeft = false;
  private static final boolean reverseOutputRight = true;

  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private PIDController leftPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController rightPID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private double basePositionLeft = 0.0;
  private double basePositionRight = 0.0;
  private double appliedVoltsLeft = 0.0;
  private double appliedVoltsRight = 0.0;

  private RomiGyro gyro = new RomiGyro();

  /** Creates a new DriveTrainIORomi. */
  public DriveTrainIORomi() {
    leftMotor.setInverted(reverseOutputLeft);
    rightMotor.setInverted(reverseOutputRight);
    gyro.reset();
  }

  public void updateInputs(DriveTrainIOInputs inputs) {
    if (closedLoop) {
      double leftVolts = leftPID.calculate(leftEncoder.getRate() * radiansPerTick) + leftFFVolts;
      double rightVolts = rightPID.calculate(rightEncoder.getRate() * radiansPerTick) + rightFFVolts;
      appliedVoltsLeft = leftVolts;
      appliedVoltsRight = rightVolts;
      leftMotor.setVoltage(leftVolts);
      rightMotor.setVoltage(rightVolts);
    }

    inputs.leftPositionRad = (leftEncoder.getDistance() * radiansPerTick) - basePositionLeft;
    inputs.leftVelocityRadPerSec = leftEncoder.getRate() * radiansPerTick;
    inputs.leftAppliedVolts = appliedVoltsLeft;
    inputs.leftCurrentAmps = new double[] {};
    inputs.leftTempCelcius = new double[] {};

    inputs.rightPositionRad = (rightEncoder.getDistance() * radiansPerTick) - basePositionRight;
    inputs.rightVelocityRadPerSec = rightEncoder.getRate() * radiansPerTick;
    inputs.rightAppliedVolts = appliedVoltsRight;
    inputs.rightCurrentAmps = new double[] {};
    inputs.rightTempCelcius = new double[] {};

    inputs.gyroPositionRad = Math.toRadians(gyro.getAngleZ());
    inputs.gyroVelocityRadPerSec = Math.toRadians(gyro.getRateZ());
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    appliedVoltsLeft = leftVolts;
    appliedVoltsRight = rightVolts;
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  public void setVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec, double leftFFVolts,
      double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftVelocityRadPerSec);
    rightPID.setSetpoint(rightVelocityRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
  }

  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);
    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }

  public void resetPosition(double leftPositionRad, double rightPositionRad) {
    basePositionLeft = leftPositionRad - (leftEncoder.getDistance() * radiansPerTick);
    basePositionRight = rightPositionRad - (rightEncoder.getDistance() * radiansPerTick);
  }
}
