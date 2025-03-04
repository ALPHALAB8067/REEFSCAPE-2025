package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;
import frc.robot.util.WristConstants;

public class WristGoToStraightCMD extends Command {

  private final WristSS wristSS;

  public WristGoToStraightCMD(WristSS pWristSS) {

    wristSS = pWristSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSS.goToStraight();
  }

  @Override
  public void end(boolean interrupted) {
    wristSS.stopWrist();
  }

  @Override
  public boolean isFinished() {

    if(wristSS.getRotatePosition() == WristConstants.straightPosition) {
      return true;
    } else 
      return false;

  }
}
