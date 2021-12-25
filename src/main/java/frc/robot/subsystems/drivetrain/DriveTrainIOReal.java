// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

/** Drive subsystem hardware interface for kitbot. */
public class DriveTrainIOReal implements DriveTrainIO {

  private static final double radiansPerTick = (2.0 * Math.PI) / 1440.0;
  private static final boolean reverseOutputLeft = false;
  private static final boolean reverseOutputRight = true;
  private static final boolean reverseSensorLeft = false;
  private static final boolean reverseSensorRight = false;

  private final TalonSRX leftLeader = new TalonSRX(1);
  private final TalonSRX leftFollower = new TalonSRX(2);
  private final TalonSRX rightLeader = new TalonSRX(3);
  private final TalonSRX rightFollower = new TalonSRX(4);

  private AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

  /** Creates a new DriveTrainIOReal. */
  public DriveTrainIOReal() {
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftLeader.configVoltageCompSaturation(12);
    rightLeader.configVoltageCompSaturation(12);

    leftLeader.setInverted(reverseOutputLeft);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightLeader.setInverted(reverseOutputRight);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    leftLeader.setSensorPhase(reverseSensorLeft);
    rightLeader.setSensorPhase(reverseSensorRight);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    ahrs.zeroYaw();
  }

  public void updateInputs(DriveTrainIOInputs inputs) {
    inputs.leftPositionRad = leftLeader.getSelectedSensorPosition() * radiansPerTick;
    inputs.rightPositionRad = rightLeader.getSelectedSensorPosition() * radiansPerTick;
    inputs.leftVelocityRadPerSec = leftLeader.getSelectedSensorVelocity() * radiansPerTick * 10;
    inputs.rightVelocityRadPerSec = rightLeader.getSelectedSensorVelocity() * radiansPerTick * 10;
    inputs.leftCurrentAmps = new double[] { leftLeader.getStatorCurrent() };
    inputs.rightCurrentAmps = new double[] { rightLeader.getStatorCurrent() };
    inputs.gyroPositionRad = Math.toRadians(ahrs.getAngle());
    inputs.gyroVelocityRadPerSec = Math.toRadians(ahrs.getRawGyroZ());
  }

  public void setOutputVolts(double leftVoltage, double rightVoltage) {
    leftLeader.set(ControlMode.PercentOutput, leftVoltage / 12);
    rightLeader.set(ControlMode.PercentOutput, rightVoltage / 12);
  }
}
