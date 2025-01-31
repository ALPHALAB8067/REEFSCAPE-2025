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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.motors.SparkMaxBrushedMotorSwerve;

public class ARM_SS extends SubsystemBase {
  private final SparkMax mLeadBase;
  private final SparkMax mFollowBase;
  private final SparkMax mLeadExtension;
  private final RelativeEncoder mExtensionEncoder;
  private final SparkAbsoluteEncoder mArmEncoder;
  private final SparkClosedLoopController mExtensionPIDController;
  private final SparkClosedLoopController mRotationPIDController;
  /*private final SparkMaxConfig mLeadConfig;
  private final SparkMaxConfig mFollowConfig;
  private final SparkMaxConfig mExtensionConfig;*/


  /** Creates a new ARM_SS. */
  public ARM_SS() {
    mLeadBase = new SparkMax(12,MotorType.kBrushless);
    mFollowBase = new SparkMax(17,MotorType.kBrushless);
    mLeadExtension = new SparkMax(11,MotorType.kBrushless);

    mExtensionEncoder = mLeadExtension.getAlternateEncoder();
    mArmEncoder = mLeadBase.getAbsoluteEncoder();

    mExtensionPIDController = mLeadExtension.getClosedLoopController();
    mRotationPIDController = mLeadBase.getClosedLoopController();

   /*  mLeadConfig = new SparkMaxConfig();
      mLeadConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);
        mLeadConfig.encoder
        .positionConversionFactor(Constants.ArmConstants.ExtensionPouceParTour)
        .velocityConversionFactor(Constants.ArmConstants.ExtensionPouceParTour);
        mLeadConfig.closedLoop
        .feedbackSensor(mArmEncoder)

    mFollowConfig = new SparkMaxConfig();
      mFollowConfig
      .apply(mLeadConfig)
      .follow(mLeadBase)
      .inverted(false);
  
    mExtensionConfig = new SparkMaxConfig();
      mExtensionConfig
      .inverted(false);*/
    SmartDashboard.putNumber("extensionSetpoint", 0);
    SmartDashboard.putNumber("rotationSetpoint", 0);
    }
  public void ExtensionGoToPosition(){
    mExtensionPIDController.setReference(SmartDashboard.getNumber("extensionSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  public void RotationGoToPosition(){
    mRotationPIDController.setReference(SmartDashboard.getNumber("rotationSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  public void stopExtension(){
    mLeadExtension.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
