// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/** Drive subsystem hardware interface for WPILib drivetrain sim. */
public class DriveTrainIOSim implements DriveTrainIO {

  private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, KitbotGearing.k7p31, KitbotWheelSize.SixInch, null);

  public void updateInputs(DriveTrainInputs inputs) {
    sim.update(0.02);
    inputs.leftPositionRadians = sim.getLeftPositionMeters() / DriveTrain.wheelRadiusMeters;
    inputs.rightPositionRadians = sim.getRightPositionMeters() / DriveTrain.wheelRadiusMeters;
    inputs.leftCurrentAmps = sim.getLeftCurrentDrawAmps();
    inputs.rightCurrentAmps = sim.getRightCurrentDrawAmps();
    inputs.gyroAngleDegrees = sim.getHeading().getDegrees();
  }

  public void setOutputVolts(double leftVoltage, double rightVoltage) {
    sim.setInputs(rightVoltage, leftVoltage);
  }
}
