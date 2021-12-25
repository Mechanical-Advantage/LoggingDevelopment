// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.template;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Template hardware implementation for a closed loop subsystem. */
public class ClosedLoopIOReal implements ClosedLoopIO {
  private boolean invert = false;
  private boolean invertFollower = false;
  private int currentLimit = 30;
  private double gearRatio = 1.0;

  private CANSparkMax leader;
  private CANSparkMax follower;
  private CANEncoder encoder;
  private CANPIDController pid;

  public ClosedLoopIOReal() {
    switch (Constants.getRobot()) {
      case KITBOT:
        leader = new CANSparkMax(0, MotorType.kBrushless);
        follower = new CANSparkMax(1, MotorType.kBrushless);
        invert = false;
        invertFollower = false;
        currentLimit = 30;
        gearRatio = 1.0;
        break;
    }

    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    follower.follow(leader, invertFollower);
    leader.setInverted(invert);
    leader.setSmartCurrentLimit(currentLimit);
    follower.setSmartCurrentLimit(currentLimit);
    leader.enableVoltageCompensation(12.0);
    follower.enableVoltageCompensation(12.0);

    encoder = leader.getEncoder();
    pid = leader.getPIDController();

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);
    leader.burnFlash();
    follower.burnFlash();
  }

  public void updateInputs(ClosedLoopIOInputs inputs) {
    inputs.positionRad = encoder.getPosition() * gearRatio * 2 * Math.PI;
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) * gearRatio;
    inputs.appliedVolts = leader.getAppliedOutput();
    inputs.currentAmps = new double[] { leader.getOutputCurrent(), follower.getOutputCurrent() };
    inputs.tempCelcius = new double[] { leader.getMotorTemperature(), follower.getMotorTemperature() };
  }

  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) / gearRatio, ControlType.kVelocity,
        0, ffVolts);
  }

  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void configurePID(double kp, double ki, double kd) {
    pid.setP(kp, 0);
    pid.setI(ki, 0);
    pid.setD(kd, 0);
    pid.setFF(0, 0);
    leader.burnFlash();
  }

  public void resetPosition(double positionRad) {
    encoder.setPosition(positionRad / (gearRatio * 2 * Math.PI));
  }
}
