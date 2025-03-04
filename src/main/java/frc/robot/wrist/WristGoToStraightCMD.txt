package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

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
    return false;
  }
}
