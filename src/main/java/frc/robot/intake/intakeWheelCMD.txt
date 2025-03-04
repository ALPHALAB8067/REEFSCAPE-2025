package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSS;

public class intakeWheelCMD extends Command {

  private final IntakeSS intakeSS;

  public intakeWheelCMD(IntakeSS pIntakeSS) {
    intakeSS = pIntakeSS;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSS.wheelTurn();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

public Command until(boolean intaked) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'until'");
}
}
