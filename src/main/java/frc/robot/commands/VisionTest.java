// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class VisionTest extends CommandBase {
  private static final Pose2d averagePose = new Pose2d(7, 4, new Rotation2d());

  private final DriveTrain driveTrain;
  private Random random = new Random();

  /** Creates a new VisionTest. */
  public VisionTest(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    random.setSeed((long) (Timer.getFPGATimestamp() * 1000000));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d transform = new Transform2d(
        new Translation2d((random.nextDouble() - 0.5) * 2 * 0.2,
            (random.nextDouble() - 0.5) * 2 * 0.2),
        driveTrain.getPose().getRotation());
    driveTrain.addVisionMeasurement(averagePose.transformBy(transform),
        Timer.getFPGATimestamp());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
