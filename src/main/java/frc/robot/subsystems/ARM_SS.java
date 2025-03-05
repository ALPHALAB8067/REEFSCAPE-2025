// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PositionsDictionnary;


public class ARM_SS extends SubsystemBase {
  private final SparkMax mLeadBase;
 // private final SparkMax mFollowBase;
  private final SparkMax mLeadExtension;
  private final SparkMax mWristMotor;
  private final RelativeEncoder mExtensionEncoder;
 // private final SparkAbsoluteEncoder mArmEncoder;
  private final RelativeEncoder mArmEncoder;
  private final RelativeEncoder mWristEncoder;
  private final SparkClosedLoopController mExtensionPIDController;
  private final SparkClosedLoopController mArmPIDControler;
  private final SparkClosedLoopController mWristPIDController;
  private final SparkMaxConfig mLeadConfig;
  //private final SparkMaxConfig mFollowConfig;
  private final SparkMaxConfig mExtensionConfig;
  private boolean currentlyRunning = false;
  private boolean done = false;
  private final SparkMaxConfig mWristConfig;
  public PositionType_SS currentPostion;

  /** Creates a new ARM_SS. */
  public ARM_SS() {
    //ARM MOTORS
    mLeadBase = new SparkMax(12,MotorType.kBrushless);
    mLeadConfig = new SparkMaxConfig();
      mLeadConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);
      mLeadConfig.encoder
      .positionConversionFactor(Constants.ArmConstants.RotationdegresParTour)
      .velocityConversionFactor(Constants.ArmConstants.RotationdegresParTour);
      mLeadConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.003,ClosedLoopSlot.kSlot0)//p = 0.003
      .i(0,ClosedLoopSlot.kSlot0)//i = 0.000001
      .d(0,ClosedLoopSlot.kSlot0);//d = 0.0001
    mLeadBase.configure(mLeadConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    /*mFollowBase = new SparkMax(17,MotorType.kBrushless);
      mFollowConfig = new SparkMaxConfig();
      mFollowConfig
      .follow(mLeadBase);
    mFollowBase.configure(mFollowConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
*/
    //EXTENSION MOTORS
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

    //WRIST MOTOR
    mWristMotor = new SparkMax(17,MotorType.kBrushless);
      mWristConfig = new SparkMaxConfig();
        mWristConfig.idleMode(IdleMode.kBrake);
      mWristConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
      mWristConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(0.0000000001,0,0,ClosedLoopSlot.kSlot0);
    mWristMotor.configure(mWristConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
      



    mExtensionEncoder = mLeadExtension.getEncoder();
    mArmEncoder = mLeadBase.getEncoder();
    mWristEncoder = mWristMotor.getEncoder();

    mExtensionPIDController = mLeadExtension.getClosedLoopController();
    mArmPIDControler = mLeadBase.getClosedLoopController();
    mWristPIDController = mWristMotor.getClosedLoopController();

    SmartDashboard.putNumber("extensionSetpoint", 0);
    SmartDashboard.putNumber("rotationSetpoint", 0);
    SmartDashboard.putNumber("WristSetpoint", 0);

  }

 /*  public void ExtensionGoToPosition(){
    mExtensionPIDController.setReference( SmartDashboard.getNumber("extensionSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void RotationGoToPosition(){    
    mArmPIDControler.setReference(SmartDashboard.getNumber("rotationSetpoint", 0) + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void WristGoToPosition(){
    mExtensionPIDController.setReference(SmartDashboard.getNumber("WristSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  */
 // public void AllInOne(/*double pLongueur, double ppPosition.armAngle*//*, parmAngleWrist */){
 //   RotationGoToPosition();
 //   ExtensionGoToPosition();
 //   WristGoToPosition();
 //}

  public void restart(){
   currentlyRunning = false;
   done = false;
  }

  public boolean isDone(){
    return done;
  }

  public void change_position_3steps(double armAngle, double longueur, double threshold){//threshold is not used curently but might be usefull
    if(currentlyRunning == false){
      mExtensionPIDController.setReference(0, ControlType.kPosition,ClosedLoopSlot.kSlot0);
      currentlyRunning = true;
    }
    else if(currentlyRunning==true){
            if (mExtensionEncoder.getPosition() <= 0 + 2 ){
                mArmPIDControler.setReference(armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
            if(mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone >= armAngle - 10 && mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone <= armAngle + 10){
                mExtensionPIDController.setReference(longueur, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                //currentlyRunning = false;
                if ((mExtensionEncoder.getPosition() >= longueur-1) && (mExtensionEncoder.getPosition() <= longueur + 1) ){
                  done = true;
                  currentlyRunning = false;
                }
              }

      }
    }

    public boolean isArmInPosition (double wantedarmAngle,double tolerance){
      return (mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone >= wantedarmAngle - tolerance && mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderSafeZone <= wantedarmAngle + tolerance);
    }
    public boolean isLenghtInPostition(double wantedArmLength,double tolerance){
      return (mExtensionEncoder.getPosition() >= wantedArmLength - tolerance && mExtensionEncoder.getPosition() <= wantedArmLength + tolerance);
    }
    public boolean isWristInPosition(double wantedarmAngle,double tolerance){
      return (mWristEncoder.getPosition() - Constants.ArmConstants.WristEncoderSafeZone >= wantedarmAngle - tolerance && mWristEncoder.getPosition() - Constants.ArmConstants.WristEncoderSafeZone <= wantedarmAngle + tolerance);
    }

   /*  public void strategie1A(double pPosition.armAngle,double pPosition.armTolerance, double pPosition.armLength,double pPosition.lenghtTolerance,double pPosition.wrist ,double pPosition.wristTolerance,double rotationWrist){
      if(currentlyRunning == false){
        mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        currentlyRunning = true;
      }
      else if(currentlyRunning==true){
              if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                  mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
              if(isWristInPosition(pPosition.wrist,pPosition.wristTolerance)){
                  mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                  
                   if (isLenghtInPostition(pPosition.armLength, pPosition.wristTolerance) ){
                    done = true;
                    currentlyRunning = false;
                  }
                }
  
        }
      }
*/


        //arm -> wrist -> longueur 
      public void S1A(PositionType_SS pPosition){
        if(currentlyRunning == false){
          mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
          currentlyRunning = true;
        }
        else if(currentlyRunning==true){
          if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
            done = true;
            currentlyRunning = false;
            }
            else if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
              mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
              else if(isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
    
          }
        }

        //armAngle -> wrist + longueur
      public void S1B(PositionType_SS pPosition){
          if(currentlyRunning == false){
            mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
            currentlyRunning = true;
          }
          else if(currentlyRunning==true){
            //is everything in position
            if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
              done = true;
              currentlyRunning = false;
              }
              //if arm is in position start wrist and pPosition.armLength
              else if(isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
            }
                
            }
          
        //wrist -> longueur + armAngle
      public void S2B(PositionType_SS pPosition){
        if(currentlyRunning == false){
          //start wrist
          mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
          currentlyRunning = true;
        }
        else if(currentlyRunning==true){
          //is everything in position
          if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
            done = true;
            currentlyRunning = false;
            } 
            // if wrist is in position start pPosition.armLength and armAngle
            else if(isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
            mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
            mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
            }
        }
      }

        //longueur + wrist -> armAngle
      public void S3(PositionType_SS pPosition){
          if(currentlyRunning == false){
            mExtensionPIDController.setReference(pPosition.armLength , ControlType.kPosition,ClosedLoopSlot.kSlot0);
            mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
            currentlyRunning = true;
          }
          else if(currentlyRunning==true){
            //is everything in position
            if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
              done = true;
              currentlyRunning = false;
              }
              else if(isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
                mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
                  }
                
            }

        //armAngle -> longueur -> wrist
      public void S4(PositionType_SS pPosition){
        if(currentlyRunning == false){
          mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderSafeZone, ControlType.kPosition,ClosedLoopSlot.kSlot0);
          currentlyRunning = true;
        }
        else if(currentlyRunning==true){
          if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
            done = true;
            currentlyRunning = false;
            }
            else if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
              mWristPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
              else if(isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                mExtensionPIDController.setReference(pPosition.armLength , ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
    
          }
        }
      
 
      public PositionType_SS whereAmI (){
        return currentPostion; 
      }
    
      public void imHere(PositionType_SS pPostion){
        currentPostion = pPostion;
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
    SmartDashboard.putNumber("actual Wrist Position", mWristEncoder.getPosition());

    SmartDashboard.putNumber("Extension output", mLeadExtension.getAppliedOutput());
    SmartDashboard.putNumber("Wrist output", mWristMotor.getAppliedOutput());
    SmartDashboard.putNumber("Rotation output", mLeadBase.getAppliedOutput());

    SmartDashboard.putBoolean("done", done);
  }
}
