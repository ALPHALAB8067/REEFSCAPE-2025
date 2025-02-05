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


public class ARM_SS extends SubsystemBase {
  private final SparkMax mLeadBase;
  private final SparkMax mFollowBase;
  private final SparkMax mLeadExtension;
  private final RelativeEncoder mExtensionEncoder;
  private final SparkAbsoluteEncoder mArmEncoder;
  private final SparkClosedLoopController mExtensionPIDController;
  private final SparkClosedLoopController mRotationPIDController;
  private final SparkMaxConfig mLeadConfig;
  private final SparkMaxConfig mFollowConfig;
  private final SparkMaxConfig mExtensionConfig;
  private double wanted;



  /** Creates a new ARM_SS. */
  public ARM_SS() {
    SmartDashboard.putNumber("x", wanted);

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
      .p(0.003000000026077032)//p = 0.003
      .i( 9.999999974752427e-7)//i = 0.000001
      .d(0.00009999999747378752);//d = 0.0001
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
    }
  public void ExtensionGoToPosition(){
    mExtensionPIDController.setReference( SmartDashboard.getNumber("extensionSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  public void RotationGoToPosition(){

    
    mRotationPIDController.setReference(SmartDashboard.getNumber("rotationSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  public void stopExtension(){
    mLeadExtension.set(0);
  }
  @Override
  public void periodic() {

    SmartDashboard.putNumber("actual position", mExtensionEncoder.getPosition());
    SmartDashboard.putNumber("output", mLeadExtension.getAppliedOutput());
    // This method will be called once per scheduler run
  }
}
