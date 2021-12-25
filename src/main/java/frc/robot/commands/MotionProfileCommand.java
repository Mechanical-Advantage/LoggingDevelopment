// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class MotionProfileCommand extends CommandBase {

  private static final double ramseteB = 2;
  private static final double ramseteZeta = 0.7;
  private static final double maxVoltage = 10.0;

  private final DriveTrain driveTrain;
  private final List<Pose2d> waypoints;
  private final double endVelocityMetersPerSec;
  private final boolean reversed;

  private Timer timer = new Timer();
  private DifferentialDriveKinematics kinematics;
  private Trajectory trajectory;
  private RamseteController ramseteController = new RamseteController(ramseteB, ramseteZeta);

  /** Creates a new MotionProfileCommand. */
  public MotionProfileCommand(DriveTrain driveTrain, List<Pose2d> waypoints, double endVelocityMetersPerSec,
      boolean reversed) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.waypoints = waypoints;
    this.endVelocityMetersPerSec = endVelocityMetersPerSec;
    this.reversed = reversed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kinematics = new DifferentialDriveKinematics(driveTrain.getTrackWidthMeters());
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(driveTrain.getKS(), driveTrain.getKV(), driveTrain.getKA()), kinematics, maxVoltage);
    double startVelocityMetersPerSec = (driveTrain.getLeftVelocityMetersPerSec()
        + driveTrain.getRightVelocityMetersPerSec()) / 2;
    TrajectoryConfig config = new TrajectoryConfig(driveTrain.getMaxVelocityMetersPerSec(),
        driveTrain.getMaxAccelerationMetersPerSecSq()).setKinematics(kinematics)
            .setStartVelocity(startVelocityMetersPerSec).setEndVelocity(endVelocityMetersPerSec).setReversed(reversed)
            .addConstraint(voltageConstraint);

    config.addConstraint(new CentripetalAccelerationConstraint(0.5));

    ArrayList<Pose2d> allWaypoints = new ArrayList<>();
    allWaypoints.add(driveTrain.getPose());
    allWaypoints.addAll(waypoints);
    trajectory = TrajectoryGenerator.generateTrajectory(allWaypoints, config);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State setpoint = trajectory.sample(timer.get());
    ChassisSpeeds chassisSpeeds = ramseteController.calculate(driveTrain.getPose(), setpoint);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    driveTrain.driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

    Logger.getInstance().recordOutput("MotionProfile/RotationSetpointDegrees",
        setpoint.poseMeters.getRotation().getDegrees());
    Logger.getInstance().recordOutput("MotionProfile/RotationDegrees",
        driveTrain.getPose().getRotation().getDegrees());
    Logger.getInstance().recordOutput("MotionProfile/XSetpointMeters",
        setpoint.poseMeters.getX());
    Logger.getInstance().recordOutput("MotionProfile/YSetpointMeters",
        setpoint.poseMeters.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
