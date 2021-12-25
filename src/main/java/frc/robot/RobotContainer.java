// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ElevatorTest;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.elevator.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain;
  private final Elevator elevator;

  private final XboxController controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate subsystems
    if (Constants.getMode() == Mode.REPLAY) {
      driveTrain = new DriveTrain(new DriveTrainIO() {
      });
      elevator = new Elevator(new ElevatorIO() {
      });
    } else {
      switch (Constants.getRobot()) {
        case KITBOT:
          driveTrain = new DriveTrain(new DriveTrainIOReal());
          elevator = new Elevator(new ElevatorIO() {
          });
          break;

        case SIMBOT:
          driveTrain = new DriveTrain(new DriveTrainIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          break;

        case ROMI:
          driveTrain = new DriveTrain(new DriveTrainIORomi());
          elevator = new Elevator(new ElevatorIO() {
          });
          break;

        default:
          driveTrain = new DriveTrain(new DriveTrainIO() {
          });
          elevator = new Elevator(new ElevatorIO() {
          });
          break;

      }
    }

    // Set up default commands
    driveTrain.setDefaultCommand(
        new DriveWithJoysticks(driveTrain, () -> controller.getRawAxis(1), () -> controller.getRawAxis(0)));
    elevator.setDefaultCommand(new ElevatorTest(elevator));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new SysIdCommand(driveTrain, driveTrain::driveVoltage,
    // driveTrain::getSysIdData);
    return new InstantCommand(() -> driveTrain.setPose(new Pose2d())).andThen(
        new MotionProfileCommand(driveTrain,
            List.of(new Pose2d(0.25, -0.4, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, -0.8, Rotation2d.fromDegrees(-135)),
                new Pose2d(-0.25, -1.2, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, -1.6, Rotation2d.fromDegrees(0)),
                new Pose2d(0.25, -1.2, Rotation2d.fromDegrees(90)),
                new Pose2d(0, -0.8, Rotation2d.fromDegrees(135)),
                new Pose2d(-0.25, -0.4, Rotation2d.fromDegrees(90)),
                new Pose2d(0.25, 0, Rotation2d.fromDegrees(0))),
            0.0, false));
  }
}
