package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSS;
import frc.robot.subsystems.WristSS;
import frc.robot.commands.intake.*;
import frc.robot.commands.wrist.WristGoToAngledCMD;
import frc.robot.commands.wrist.WristIntakeCMD;

//sequential bc we can't make all the commands at once. 
//we can set parallel command groups in a sequence
public class getCoralCMD extends SequentialCommandGroup {

  private final IntakeSS intake;
  private final WristSS wrist;

  public getCoralCMD(IntakeSS pIntakeSS, WristSS pWristss) {

    intake = pIntakeSS;
    wrist = pWristss;

    addCommands(
      
    new ParallelCommandGroup(
      new intakeWheelCMD(intake).until(intake.getIntaked() == true),
      new WristGoToAngledCMD(wrist),
      new intakeDownCMD(intake)
    ),

    new ParallelCommandGroup(
      new intakeUpCMD(intake)
      //bring arm to upped intake
    ),

    new WristIntakeCMD(wrist).until(wrist.hasSomething() == true)

    );

    
  }
}
