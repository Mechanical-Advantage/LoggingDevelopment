// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SysIdCommand.DriveTrainSysIdData;
import frc.robot.subsystems.drivetrain.DriveTrainIO.DriveTrainIOInputs;
import frc.robot.util.TunableNumber;

public class DriveTrain extends SubsystemBase {

  private final double wheelRadiusMeters;
  private final double maxVelocityMetersPerSec;
  private final double maxAccelerationMetersPerSecSq;
  private final double trackWidthMeters;
  private final TunableNumber kP = new TunableNumber("DriveTrain/kP");
  private final TunableNumber kI = new TunableNumber("DriveTrain/kI");
  private final TunableNumber kD = new TunableNumber("DriveTrain/kD");
  private final double leftKS;
  private final double leftKV;
  private final double leftKA;
  private final double rightKS;
  private final double rightKV;
  private final double rightKA;

  private final DriveTrainIO io;
  private final DriveTrainIOInputs inputs = new DriveTrainIOInputs();

  private final SimpleMotorFeedforward leftFFModel;
  private final SimpleMotorFeedforward rightFFModel;

  private double lastKP = 0.0;
  private double lastKI = 0.0;
  private double lastKD = 0.0;

  private DifferentialDriveOdometry odometry;
  private Field2d field2d = new Field2d();
  private double baseDistanceLeft = 0.0;
  private double baseDistanceRight = 0.0;

  /** Creates a new DriveTrain. */
  public DriveTrain(DriveTrainIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case KITBOT:
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        maxVelocityMetersPerSec = 1.0;
        maxAccelerationMetersPerSecSq = 1.0;
        trackWidthMeters = 1.0;
        kP.setDefault(0.0);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        leftKS = 0.0;
        leftKV = 0.0;
        leftKA = 0.0;
        rightKS = 0.0;
        rightKV = 0.0;
        rightKA = 0.0;
        break;
      case SIMBOT:
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        maxVelocityMetersPerSec = 1.0;
        maxAccelerationMetersPerSecSq = 1.0;
        trackWidthMeters = 1.0;
        kP.setDefault(0.0);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        leftKS = 0.0;
        leftKV = 0.15455;
        leftKA = 0.026795;
        rightKS = 0.0;
        rightKV = 0.15455;
        rightKA = 0.026795;
        break;
      case ROMI:
        wheelRadiusMeters = 0.07;
        maxVelocityMetersPerSec = 0.75;
        maxAccelerationMetersPerSecSq = 0.5;
        trackWidthMeters = 0.281092;
        kP.setDefault(0.15);
        kI.setDefault(0.3);
        kD.setDefault(0.01);
        leftKS = 0.23607;
        leftKV = 0.35962;
        leftKA = 0.0;
        rightKS = 0.23136;
        rightKV = 0.41541;
        rightKA = 0.0;
        break;
      default:
        wheelRadiusMeters = Units.inchesToMeters(3.0);
        maxVelocityMetersPerSec = 1.0;
        maxAccelerationMetersPerSecSq = 1.0;
        trackWidthMeters = 1.0;
        kP.setDefault(0.0);
        kI.setDefault(0.0);
        kD.setDefault(0.0);
        leftKS = 0.0;
        leftKV = 0.0;
        leftKA = 0.0;
        rightKS = 0.0;
        rightKV = 0.0;
        rightKA = 0.0;
        break;
    }

    io.setBrakeMode(true);
    io.configurePID(kP.get(), kI.get(), kD.get());
    lastKP = kP.get();
    lastKI = kI.get();
    lastKD = kD.get();

    leftFFModel = new SimpleMotorFeedforward(leftKS, leftKV, leftKA);
    rightFFModel = new SimpleMotorFeedforward(rightKS, rightKV, rightKA);
    SmartDashboard.putData("Odometry", field2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("DriveTrain", inputs);

    Logger.getInstance().recordOutput("DriveTrain/LeftVelocityMetersPerSec", getLeftVelocityMetersPerSec());
    Logger.getInstance().recordOutput("DriveTrain/RightVelocityMetersPerSec", getRightVelocityMetersPerSec());

    if (odometry == null) {
      odometry = new DifferentialDriveOdometry(new Rotation2d(inputs.gyroPositionRad * -1));
      baseDistanceLeft = getLeftPositionMeters();
      baseDistanceRight = getRightPositionMeters();
    } else {
      Pose2d pose = odometry.update(new Rotation2d(inputs.gyroPositionRad * -1),
          getLeftPositionMeters() - baseDistanceLeft,
          getRightPositionMeters() - baseDistanceRight);
      Logger.getInstance().recordOutput("Odometry/RotationDegrees", pose.getRotation().getDegrees());
      Logger.getInstance().recordOutput("Odometry/XMeters", pose.getX());
      Logger.getInstance().recordOutput("Odometry/YMeters", pose.getY());
      field2d.setRobotPose(pose);
    }

    if (Constants.tuningMode) {
      if (kP.get() != lastKP || kI.get() != lastKI || kD.get() != lastKD) {
        io.configurePID(kP.get(), kI.get(), kD.get());
        lastKP = kP.get();
        lastKI = kI.get();
        lastKD = kD.get();
      }
    }
  }

  /**
   * Drive at the specified voltage with no other processing. Only use with SysId.
   */
  public void driveVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * Drive at the specified percentage of max speed.
   */
  public void drivePercent(double leftPercent, double rightPercent) {
    driveVelocity(leftPercent * maxVelocityMetersPerSec, rightPercent * maxVelocityMetersPerSec);
  }

  /**
   * Drive at the specified velocity.
   */
  public void driveVelocity(double leftVelocityMetersPerSec, double rightVelocityMetersPerSec) {
    double leftVelocityRadPerSec = leftVelocityMetersPerSec / wheelRadiusMeters;
    double rightVelocityRadPerSec = rightVelocityMetersPerSec / wheelRadiusMeters;
    double leftVolts = leftFFModel.calculate(leftVelocityRadPerSec);
    double rightVolts = rightFFModel.calculate(rightVelocityRadPerSec);

    if (Constants.driveTrainOpenLoop) {
      io.setVoltage(leftVolts, rightVolts);
    } else {
      io.setVelocity(leftVelocityRadPerSec, rightVelocityRadPerSec, leftVolts, rightVolts);
    }
  }

  /**
   * In open loop, goes to neutral. In closed loop, zeroes velocity setpoint.
   */
  public void stop() {
    if (Constants.driveTrainOpenLoop) {
      io.setVoltage(0.0, 0.0);
    } else {
      io.setVelocity(0.0, 0.0, 0.0, 0.0);
    }
  }

  /**
   * Returns the current odometry position.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Reset the current odometry position.
   */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(pose, new Rotation2d(inputs.gyroPositionRad * -1));
    baseDistanceLeft = getLeftPositionMeters();
    baseDistanceRight = getRightPositionMeters();
  }

  /**
   * Returns the position of the left drive in meters.
   */
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * wheelRadiusMeters;
  }

  /**
   * Returns the position of the right drive in meters.
   */
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * wheelRadiusMeters;
  }

  /**
   * Returns the velocity of the left drive in meters per second.
   */
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /**
   * Returns the velocity of the right drive in meters per second.
   */
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the maximum velocity (meters/second) */
  public double getMaxVelocityMetersPerSec() {
    return maxVelocityMetersPerSec;
  }

  /** Returns the maximum acceleration (meters/second^2) */
  public double getMaxAccelerationMetersPerSecSq() {
    return maxAccelerationMetersPerSecSq;
  }

  /** Returns the empirical track width (meters) */
  public double getTrackWidthMeters() {
    return trackWidthMeters;
  }

  /** Returns the average kS value. */
  public double getKS() {
    return (leftKS + rightKS) / 2;
  }

  /** Returns the average kV value (meters). */
  public double getKV() {
    return ((leftKV + rightKV) / 2) * wheelRadiusMeters;
  }

  /** Returns the average kA value (meters). */
  public double getKA() {
    return ((leftKA + rightKA) / 2) * wheelRadiusMeters;
  }

  /**
   * Returns a set of data for SysId
   */
  public DriveTrainSysIdData getSysIdData() {
    return new DriveTrainSysIdData(
        inputs.leftPositionRad,
        inputs.rightPositionRad,
        inputs.leftVelocityRadPerSec,
        inputs.rightVelocityRadPerSec,
        inputs.gyroPositionRad,
        inputs.gyroVelocityRadPerSec);
  }
}
