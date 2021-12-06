// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DriveTrain extends SubsystemBase {

  public static final double wheelRadiusMeters = Units.inchesToMeters(3.0);

  private final DriveTrainIO io;
  private final DriveTrainInputs inputs = new DriveTrainInputs();

  private DifferentialDriveOdometry odometry;
  private Field2d field2d = new Field2d();
  private double baseDistanceLeft = 0.0;
  private double baseDistanceRight = 0.0;

  /** Creates a new DriveTrain. */
  public DriveTrain(DriveTrainIO io) {
    this.io = io;
    SmartDashboard.putData("Odometry", field2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("DriveTrain", inputs);

    if (odometry == null) {
      odometry = new DifferentialDriveOdometry(getGyroAngle());
      baseDistanceLeft = getLeftPositionMeters();
      baseDistanceRight = getRightPositionMeters();
    } else {
      Pose2d pose = odometry.update(getGyroAngle(), getLeftPositionMeters() - baseDistanceLeft,
          getRightPositionMeters() - baseDistanceRight);
      Logger.getInstance().recordOutput("Odometry/RotationDegrees", pose.getRotation().getDegrees());
      Logger.getInstance().recordOutput("Odometry/XMeters", pose.getX());
      Logger.getInstance().recordOutput("Odometry/YMeters", pose.getY());
      field2d.setRobotPose(pose);
    }

    Logger.getInstance().recordOutput("DriveTrain/TotalCurrentAmps",
        Math.abs(inputs.leftCurrentAmps) + Math.abs(inputs.rightCurrentAmps));
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

  /**
   * Returns a Rotation2d object representing the gyro angle (using the WPILib
   * coordinate system).
   */
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(inputs.gyroAngleDegrees * -1);
  }
}
