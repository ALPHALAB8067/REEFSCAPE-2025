// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PositionsDictionnary;


public class ARM_SS extends SubsystemBase {
  private final SparkMax mLeadBase;
  private final SparkMax mFollowBase;
  private final SparkMax mLeadExtension;
 // private final SparkMax mWrist;
  private final RelativeEncoder mExtensionEncoder;
  private final SparkAbsoluteEncoder mArmEncoder;
 // private final SparkAbsoluteEncoder mWristEncoder;
  private final SparkClosedLoopController mExtensionPIDController;
  private final SparkClosedLoopController mRotationPIDController;
  //private final SparkClosedLoopController mWristPIDController;
  private final SparkMaxConfig mLeadConfig;
  private final SparkMaxConfig mFollowConfig;
  private final SparkMaxConfig mExtensionConfig;
  private boolean currentlyRunning = false;
  private boolean done = false;
 // private final SparkMaxConfig mWristConfig;
  private PositionType_SS mPositionType_SS;

  /** Creates a new ARM_SS. */
  public ARM_SS() {
    mLeadBase = new SparkMax(12,MotorType.kBrushless);
    mLeadConfig = new SparkMaxConfig();
      mLeadConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);
      mLeadConfig.encoder
      .positionConversionFactor(Constants.ArmConstants.RotationdegresParTour)
      .velocityConversionFactor(Constants.ArmConstants.RotationdegresParTour);
      mLeadConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(0.003,ClosedLoopSlot.kSlot0)//p = 0.003
      .i(0,ClosedLoopSlot.kSlot0)//i = 0.000001
      .d(0,ClosedLoopSlot.kSlot0);//d = 0.0001
    mLeadBase.configure(mLeadConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    mFollowBase = new SparkMax(17,MotorType.kBrushless);
      mFollowConfig = new SparkMaxConfig();
      mFollowConfig
      .follow(mLeadBase);
    mFollowBase.configure(mFollowConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);


    mLeadExtension = new SparkMax(11,MotorType.kBrushless);
      mExtensionConfig = new SparkMaxConfig();
      mExtensionConfig
      .idleMode(IdleMode.kCoast);
      mExtensionConfig.encoder
      .positionConversionFactor(Constants.ArmConstants.ExtensionPouceParTour)
      .velocityConversionFactor(Constants.ArmConstants.ExtensionPouceParTour);
      mExtensionConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(1, 0, 0, ClosedLoopSlot.kSlot0);
    mLeadExtension.configure(mExtensionConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    mExtensionEncoder = mLeadExtension.getAlternateEncoder();
    mArmEncoder = mLeadBase.getAbsoluteEncoder();

    mExtensionPIDController = mLeadExtension.getClosedLoopController();
    mRotationPIDController = mLeadBase.getClosedLoopController();

    SmartDashboard.putNumber("extensionSetpoint", 0);
    SmartDashboard.putNumber("rotationSetpoint", 0);
    /*SmartDashboard.putNumber("WristSetpoint", 0);*/
   // mPositionType_SS = new PositionsDictionnary.mTrajectory1.get(0);
  }

  public void ExtensionGoToPosition(){
    mExtensionPIDController.setReference( SmartDashboard.getNumber("extensionSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void RotationGoToPosition(){    
    mRotationPIDController.setReference(SmartDashboard.getNumber("rotationSetpoint", 0) + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  /*public void WristGoToPosition(){
    mExtensionPIDController.setReference(SmartDashboard.getNumber("WristSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  */
  public void AllInOne(/*double pLongueur, double pAngleBase*//*, pAngleWrist */){
    RotationGoToPosition();
    ExtensionGoToPosition();
    /*  mExtensionPIDController.setReference(pLongueur, ControlType.kPosition,ClosedLoopSlot.kSlot0);
     mRotationPIDController.setReference( pAngleBase + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
    */
     /*mWristPIDController.setReference(pAngleWrist, ControlType.kPosition,ClosedLoopSlot.kSlot0);*/
  }
  public void restart(){
   currentlyRunning = false;
   done = false;
  }
  public boolean isDone(){
    return done;
  }
  public void change_position_3steps(double angleBase, double longueur, double threshold){//threshold is not used curently but might be usefull
    if(currentlyRunning == false){
      mExtensionPIDController.setReference(0, ControlType.kPosition,ClosedLoopSlot.kSlot0);
      currentlyRunning = true;
    }
    else if(currentlyRunning==true){
            if (mExtensionEncoder.getPosition() <= 0 + 2 ){
                mRotationPIDController.setReference(angleBase + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
            if(mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone >= angleBase - 10 && mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone <= angleBase + 10){
                mExtensionPIDController.setReference(longueur, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                //currentlyRunning = false;
                if ((mExtensionEncoder.getPosition() >= longueur-1) && (mExtensionEncoder.getPosition() <= longueur + 1) ){
                  done = true;
                  currentlyRunning = false;
                }
              }

      }
    }
 

  public void ManualExtension(double pSpeed){
    mLeadExtension.set(pSpeed);
  }
  
  public void ManualRotation(double pSpeed){
    mLeadBase.set(pSpeed);
  }

  /*public void ManualWrist(double pSpeed){
    mWrist.set(pSpeed);
  }*/

  public void stopExtension(){
    mLeadExtension.set(0);
  }

  public void stopRotation(){
    mLeadBase.set(0);
  }

  /*public void stopWrist(){
    mWrist.set(0);
  }*/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("actual Extension position", mExtensionEncoder.getPosition());
    SmartDashboard.putNumber("actual rotation position", mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone);
    //SmartDashboard.putNumber("actual Wrist Position", mWristEncoder.getPosition());
    SmartDashboard.putNumber("Extension output", mLeadExtension.getAppliedOutput());
    SmartDashboard.putNumber("Rotation output", mLeadBase.getAppliedOutput());
    SmartDashboard.putBoolean("done", done);
  }
}
