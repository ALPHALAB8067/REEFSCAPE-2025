package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSS;

public class intakeWheelReverseCMD extends Command {

  private final IntakeSS intakeSS;

  public intakeWheelReverseCMD(IntakeSS pIntakeSS) {
    intakeSS = pIntakeSS;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSS.wheelReverse();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
