package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.goToL2;
import frc.robot.commands.goToRest;
import frc.robot.commands.intakeUpCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.IntakeSS;

public class closeRobotCMD extends SequentialCommandGroup {

  private final ARM_SS armSS;
  private final IntakeSS intakeSS;

  public closeRobotCMD(ARM_SS pArm_SS, IntakeSS pIntakeSS) {
    
    armSS = pArm_SS;
    intakeSS = pIntakeSS;

    addCommands(

      new intakeUpCMD(intakeSS),
      new goToRest(armSS)

    );
  }
}
