package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSS;

public class intakeUpCMD extends Command {

  private final IntakeSS intakeSS; 

  public intakeUpCMD(IntakeSS pIntakeSS) {

    intakeSS = pIntakeSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSS.intakeUp();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
