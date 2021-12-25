// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.system.plant.DCMotor;

/** Elevator subsystem hardware interface for WPILib elevator sim. */
public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim sim = new ElevatorSim(DCMotor.getNEO(2), 1.0, 0.5, Elevator.drumRadiusMeters,
      Elevator.minPositionMeters, Elevator.maxPositionMeters);
  private PIDController pid = new PIDController(0, 0, 0);
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double ffVolts = 0.0;

  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      setLimitedVoltage(pid.calculate(sim.getVelocityMetersPerSecond() / Elevator.drumRadiusMeters) + ffVolts);
    }

    sim.update(0.02);
    inputs.positionRad = sim.getPositionMeters() / Elevator.drumRadiusMeters;
    inputs.velocityRadPerSec = sim.getVelocityMetersPerSecond() / Elevator.drumRadiusMeters;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] { sim.getCurrentDrawAmps() };
  }

  private void setLimitedVoltage(double volts) {
    double limitedVolts = volts;
    limitedVolts = limitedVolts > 12 ? 12 : limitedVolts;
    limitedVolts = limitedVolts < -12 ? -12 : limitedVolts;
    appliedVolts = limitedVolts;
    sim.setInputVoltage(limitedVolts);
  }

  public void setVoltage(double volts) {
    closedLoop = false;
    setLimitedVoltage(volts);
  }

  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    this.ffVolts = ffVolts;
    pid.setSetpoint(velocityRadPerSec);
  }

  public void configurePID(double kp, double ki, double kd) {
    pid.setP(kp);
    pid.setI(ki);
    pid.setD(kd);
  }
}
