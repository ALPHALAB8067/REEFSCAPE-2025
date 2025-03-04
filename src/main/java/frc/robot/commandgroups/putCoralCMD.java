// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.WristExitCMD;
import frc.robot.commands.WristGoToStraightCMD;
import frc.robot.subsystems.WristSS;

public class putCoralCMD extends SequentialCommandGroup {

  private final WristSS wristSS;

  public putCoralCMD(WristSS pWristSS) {
    
    wristSS = pWristSS;

    addCommands(

    new ParallelCommandGroup(
      new WristGoToStraightCMD(wristSS)
      //arm to position
    ),

    new WristExitCMD(wristSS).withTimeout(1)

    );
  }
}
