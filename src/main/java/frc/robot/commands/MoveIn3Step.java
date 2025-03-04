// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ARM_SS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIn3Step extends Command {
  /** Creates a new MoveIn3Step. */
  ARM_SS mArm_SS;
  public MoveIn3Step(ARM_SS pArm_SS) {
    mArm_SS = pArm_SS;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mArm_SS.change_position_3steps(30, 5, 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm_SS.restart();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArm_SS.isDone();
  }
}
