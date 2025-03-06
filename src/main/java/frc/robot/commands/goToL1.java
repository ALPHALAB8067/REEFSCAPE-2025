// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionsDictionnary;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.PositionType_SS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToL1 extends Command {
  ARM_SS mArm_SS;
  private PositionType_SS mTarget = PositionsDictionnary.mPositionL1;
  PositionType_SS mCurrent;
  /** Creates a new goToRest. */
  public goToL1(ARM_SS pArm_SS) {
    mArm_SS = pArm_SS;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mCurrent = mArm_SS.whereAmI();
    if(mCurrent == PositionsDictionnary.mPositionRest){  
      mArm_SS.S4(mTarget);   
    }
    if(mCurrent == PositionsDictionnary.mPositionL1){
      //does nothing 
      mArm_SS.S1A(mTarget);
    }
    if(mCurrent == PositionsDictionnary.mPositionL2){
      mArm_SS.S1A(mTarget);
    }
    if(mCurrent == PositionsDictionnary.mPositionL3){
      mArm_SS.S3(mTarget);
    }
    if(mCurrent == PositionsDictionnary.mPositionL4){
      mArm_SS.S3(mTarget);
    }
    if (mCurrent == PositionsDictionnary.mPositionCoralStation){
      mArm_SS.S3(mTarget);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("didit", true);
    mArm_SS.imHere(mTarget);
    mArm_SS.done = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArm_SS.isDone();
  }
}
