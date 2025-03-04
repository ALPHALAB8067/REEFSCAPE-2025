package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class WristExitCMD extends Command {

  private final WristSS wristSS;

  public WristExitCMD(WristSS pWristSS) {
      
    wristSS = pWristSS;
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSS.reverseWheel();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
