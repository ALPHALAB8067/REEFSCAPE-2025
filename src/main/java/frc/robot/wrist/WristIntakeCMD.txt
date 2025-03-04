package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class WristIntakeCMD extends Command {

  private final WristSS wristSS;

  public WristIntakeCMD(WristSS pWristSS) {
      
    wristSS = pWristSS;
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSS.turnWheel();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

public Command until(boolean b) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'until'");
}
}
