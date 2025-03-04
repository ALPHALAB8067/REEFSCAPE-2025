// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionType_SS extends SubsystemBase {
  /** Creates a new PositionDictionnary_SS. */
  double angle;
  double length;
  double threshold;
  public PositionType_SS(double pAngle, double pLength, double pThreshold) {
    this.angle = pAngle;
    this.length = pLength;
    this.threshold = pThreshold;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
