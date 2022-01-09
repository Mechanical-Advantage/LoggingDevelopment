// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorTest extends CommandBase {
  private final Elevator elevator;
  private double lastUpdate = 0.0;
  private Random random = new Random();

  /** Creates a new DriveElevatorWithJoystick. */
  public ElevatorTest(Elevator elevator) {
    addRequirements(elevator);
    this.elevator = elevator;
    random.setSeed((long) (Timer.getFPGATimestamp() * 1000000));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - lastUpdate > 2.0) {
      elevator.setPosition(random.nextDouble() * 3);
      lastUpdate = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.runVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
