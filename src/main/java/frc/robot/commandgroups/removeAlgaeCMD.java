package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WristExitCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.WristSS;

public class removeAlgaeCMD extends SequentialCommandGroup {

  private final WristSS wristSS;
  private final ARM_SS armSS;

  public removeAlgaeCMD(WristSS pWristSS, ARM_SS pArm_SS) {
    
    wristSS = pWristSS;
    armSS = pArm_SS;

    addCommands(

      new //bringarmtoalgae(armSS)
      .andThen(new WristExitCMD(wristSS).withTimeout(5)
      )


    );

  }
}