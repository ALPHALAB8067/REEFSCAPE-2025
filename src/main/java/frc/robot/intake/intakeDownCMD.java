package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSS;

public class intakeDownCMD extends Command {
  private final IntakeSS intakeSS; 

  public intakeDownCMD(IntakeSS pIntakeSS) {
  
    intakeSS = pIntakeSS;
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSS.intakeDown();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
