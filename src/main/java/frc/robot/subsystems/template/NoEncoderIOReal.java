// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.template;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

/** Template hardware implementation for an Open loop subsystem. */
public class NoEncoderIOReal implements NoEncoderIO {
  private boolean invert = false;
  private boolean invertFollower = false;
  private int currentLimit = 30;

  private CANSparkMax leader;
  private CANSparkMax follower;

  public NoEncoderIOReal() {
    switch (Constants.getRobot()) {
      case KITBOT:
        leader = new CANSparkMax(0, MotorType.kBrushless);
        follower = new CANSparkMax(1, MotorType.kBrushless);
        invert = false;
        invertFollower = false;
        currentLimit = 30;
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

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);
    leader.burnFlash();
    follower.burnFlash();
  }

  public void updateInputs(NoEncoderIOInputs inputs) {
    inputs.appliedVolts = leader.getAppliedOutput();
    inputs.currentAmps = new double[] { leader.getOutputCurrent(), follower.getOutputCurrent() };
    inputs.tempCelcius = new double[] { leader.getMotorTemperature(), follower.getMotorTemperature() };
  }

  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    follower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
