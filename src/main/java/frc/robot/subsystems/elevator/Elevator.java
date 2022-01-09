// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SysIdCommand.MechanismSysIdData;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {

  public static final double drumRadiusMeters = 0.03;
  public static final double minPositionMeters = 0.0;
  public static final double maxPositionMeters = 3.0;
  public static final ElevatorFeedforward ffModel = new ElevatorFeedforward(-0.00027881, 0.33923, 0.019843, 0.0010607);
  public static final double velocityKp = 0.04;
  public static final double velocityKi = 0;
  public static final double velocityKd = 0;

  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private final ProfiledPIDController pid = new ProfiledPIDController(35, 0, 0, new Constraints(20, 30));
  private boolean closedLoopPosition = false;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
    io.configurePID(velocityKp, velocityKi, velocityKd);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
    Logger.getInstance().recordOutput("Elevator/PositionMeters", getPositionMeters());
    Logger.getInstance().recordOutput("Elevator/VelocityMetersPerSec", getVelocityMetersPerSec());

    if (closedLoopPosition) {
      double velocityMetersPerSec = pid.calculate(getPositionMeters());
      double velocityRadPerSec = velocityMetersPerSec / drumRadiusMeters;
      io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

      Logger.getInstance().recordOutput("Elevator/PositionSetpointMeters", pid.getSetpoint().position);
      Logger.getInstance().recordOutput("Elevator/PositionGoalMeters", pid.getGoal().position);
      Logger.getInstance().recordOutput("Elevator/VelocitySetpointMetersPerSec", velocityMetersPerSec);
    }
  }

  public void runVoltage(double volts) {
    closedLoopPosition = false;
    io.setVoltage(volts);
  }

  public void setVelocity(double velocityMetersPerSec) {
    closedLoopPosition = false;
    double velocityRadPerSec = velocityMetersPerSec / drumRadiusMeters;
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
  }

  public void setPosition(double positionMeters) {
    if (!closedLoopPosition) {
      closedLoopPosition = true;
      pid.reset(new State(getPositionMeters(), getVelocityMetersPerSec()));
    }
    if (positionMeters < minPositionMeters) {
      positionMeters = minPositionMeters;
    }
    if (positionMeters > maxPositionMeters) {
      positionMeters = maxPositionMeters;
    }
    pid.setGoal(positionMeters);
  }

  public double getPositionMeters() {
    return inputs.positionRad * drumRadiusMeters;
  }

  public double getVelocityMetersPerSec() {
    return inputs.velocityRadPerSec * drumRadiusMeters;
  }

  public MechanismSysIdData getSysIdData() {
    return new MechanismSysIdData(inputs.positionRad, inputs.velocityRadPerSec);
  }
}
