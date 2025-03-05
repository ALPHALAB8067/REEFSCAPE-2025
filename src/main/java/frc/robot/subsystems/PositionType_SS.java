// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionType_SS  {
  /** Creates a new PositionDictionnary_SS. */
  double armAngle;
  double armLength;
  double wrist;
  double armTolerance;
  double lenghtTolerance;
  double wristTolerance;
  double claw;
  String name;

  public PositionType_SS(double pAngle, double pWrist, double pLength , double pArmTolerance, double pWristTolerance, double pLenghtTolerance, double pClaw, String pName) {
    this.armAngle = pAngle;
    this.armLength = pLength;
    this.wrist = pWrist;
    this.armTolerance = pArmTolerance;
    this.lenghtTolerance = pLenghtTolerance;
    this.wristTolerance = pWristTolerance;
    this.claw = pClaw;
    this.name = pName;
    
  }


}
