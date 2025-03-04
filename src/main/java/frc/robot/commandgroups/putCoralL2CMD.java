// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.WristExitCMD;
import frc.robot.commands.WristGoToStraightCMD;
import frc.robot.commands.goToL2;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.WristSS;

public class putCoralL2CMD extends SequentialCommandGroup {

  private final WristSS wristSS;
  private final ARM_SS armss;

  public putCoralL2CMD(WristSS pWristSS, ARM_SS pArm_SS) {
    
    wristSS = pWristSS;
    armss = pArm_SS;

    addCommands(

    new ParallelCommandGroup(
      new WristGoToStraightCMD(wristSS),
      new goToL2(armss)
    ),

    new WristExitCMD(wristSS).withTimeout(1)

    );
  }
}
