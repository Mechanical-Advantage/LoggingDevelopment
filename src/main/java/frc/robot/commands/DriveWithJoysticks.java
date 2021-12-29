// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
  private static final double maxSpeed = 1.0;
  private static final double deadband = 0.05;
  private static final double curvatureTurnSensitivity = 1.0; // Greater than 1 allows for reverse output on inner side
  private static final double hybridCurvatureThreshold = 0.15; // Under this base speed, blend to split arcade

  private final DriveTrain driveTrain;
  private final Supplier<Double> leftX;
  private final Supplier<Double> leftY;
  private final Supplier<Double> rightX;
  private final Supplier<Double> rightY;

  private final SendableChooser<JoystickMode> modeChooser = new SendableChooser<JoystickMode>();

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(DriveTrain driveTrain, Supplier<Double> leftX, Supplier<Double> leftY,
      Supplier<Double> rightX, Supplier<Double> rightY) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.rightY = rightY;

    modeChooser.addOption("Tank", JoystickMode.TANK);
    modeChooser.addOption("Split Arcade", JoystickMode.SPLIT_ARCADE);
    modeChooser.setDefaultOption("Curvature", JoystickMode.CURVATURE);
    SmartDashboard.putData("Joystick Mode", modeChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  /** Apply deadband and square output. */
  private double processJoystickAxis(double axis) {
    if (Math.abs(axis) > deadband) {
      double adjustedValue = Math.copySign((Math.abs(axis) - deadband) / (1 - deadband), axis);
      return adjustedValue * Math.abs(adjustedValue);
    } else {
      return 0;
    }
  }

  /** Represents a left and right percentage. */
  private static class WheelSpeeds {
    public double left;
    public double right;

    public WheelSpeeds(double left, double right) {
      this.left = left;
      this.right = right;
    }

    public static WheelSpeeds fromArcade(double baseSpeed, double turnSpeed) {
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }

    public static WheelSpeeds fromCurvature(double baseSpeed, double turnSpeed) {
      double maxBaseSpeed = 1 / (1 + (Math.abs(turnSpeed) * curvatureTurnSensitivity)); // Max speed where no output >1
      double baseSpeedLimited = MathUtil.clamp(baseSpeed, maxBaseSpeed * -1, maxBaseSpeed);
      turnSpeed = Math.abs(baseSpeedLimited) * turnSpeed * curvatureTurnSensitivity;
      return new WheelSpeeds(baseSpeedLimited + turnSpeed, baseSpeedLimited - turnSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftXValue = processJoystickAxis(leftX.get());
    double leftYValue = processJoystickAxis(leftY.get());
    double rightXValue = processJoystickAxis(rightX.get());
    double rightYValue = processJoystickAxis(rightY.get());

    Logger.getInstance().recordOutput("DriveWithJoysticks/LeftX", leftXValue);
    Logger.getInstance().recordOutput("DriveWithJoysticks/LeftY", leftYValue);
    Logger.getInstance().recordOutput("DriveWithJoysticks/RightX", rightXValue);
    Logger.getInstance().recordOutput("DriveWithJoysticks/RightY", rightYValue);

    WheelSpeeds outputSpeeds = new WheelSpeeds(0, 0);
    switch (modeChooser.getSelected()) {
      case TANK:
        outputSpeeds = new WheelSpeeds(leftYValue, rightYValue);
        break;
      case SPLIT_ARCADE:
        outputSpeeds = WheelSpeeds.fromArcade(leftYValue, rightXValue);
        break;
      case CURVATURE:
        WheelSpeeds splitArcadeSpeeds = WheelSpeeds.fromArcade(leftYValue, rightXValue);
        WheelSpeeds curvatureSpeeds = WheelSpeeds.fromCurvature(leftYValue, rightXValue);

        double hybridScale = Math.abs(leftYValue) / hybridCurvatureThreshold;
        hybridScale = hybridScale > 1 ? 1 : hybridScale;
        outputSpeeds = new WheelSpeeds(
            (curvatureSpeeds.left * hybridScale) + (splitArcadeSpeeds.left * (1 - hybridScale)),
            (curvatureSpeeds.right * hybridScale) + (splitArcadeSpeeds.right * (1 - hybridScale)));
        break;
    }

    Logger.getInstance().recordOutput("DriveWithJoysticks/OutputLeft", outputSpeeds.left);
    Logger.getInstance().recordOutput("DriveWithJoysticks/OutputRight", outputSpeeds.right);
    driveTrain.drivePercent(outputSpeeds.left * maxSpeed, outputSpeeds.right * maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drivePercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static enum JoystickMode {
    TANK, SPLIT_ARCADE, CURVATURE
  }
}
