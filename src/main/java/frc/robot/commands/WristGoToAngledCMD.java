package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;
import frc.robot.util.WristConstants;

public class WristGoToAngledCMD extends Command {

  private final WristSS wristSS;

  public WristGoToAngledCMD(WristSS pWristSS) {

    wristSS = pWristSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSS.goToAngled();
  }

  @Override
  public void end(boolean interrupted) {
    wristSS.stopWrist();
  }

  @Override
  public boolean isFinished() {
    
    if(wristSS.getRotatePosition() == WristConstants.angledPosition) {
      return true;
    } else 
      return false;
    
  }
}
