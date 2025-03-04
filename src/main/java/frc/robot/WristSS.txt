package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.WristConstants;

public class WristSS extends SubsystemBase {

  private final SparkMax rotateSparkMax;
  private final SparkMax wheelSparkMax;

  private final SparkMaxConfig rotateConfig;
  private final SparkMaxConfig wheelConfig;

  private final SparkAbsoluteEncoder thru;

  private final SparkClosedLoopController rotatePID;

  private final DigitalInput digitalInput;
  

  public WristSS() {

    rotateSparkMax = new SparkMax(WristConstants.rotateSparkMaxPort, MotorType.kBrushless);

    rotateConfig = new SparkMaxConfig();
    rotateConfig.inverted(WristConstants.rotateIsInverted);
    rotateConfig.idleMode(WristConstants.rotateIdleMode);
    rotateConfig.encoder.positionConversionFactor(WristConstants.positionConversionFactor);
    rotateConfig.closedLoop
    .feedbackSensor(WristConstants.feedbacksensor)
    .p(WristConstants.pValue)
    .i(WristConstants.iValue)
    .d(WristConstants.dValue);

    wheelSparkMax = new SparkMax(WristConstants.wheelSparkMaxPort, MotorType.kBrushless);

    wheelConfig = new SparkMaxConfig();
    wheelConfig.inverted(WristConstants.wheelIsInverted);
    wheelConfig.idleMode(WristConstants.wheelIdleMode);

    thru = rotateSparkMax.getAbsoluteEncoder();

    rotatePID = rotateSparkMax.getClosedLoopController();

    digitalInput = new DigitalInput(WristConstants.digitalinputport);

  }

  public double getRotatePosition() {
    return 360 * thru.getPosition();
  }

  public void goToStraight() {
    rotatePID.setReference(WristConstants.straightPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void goToAngled() {
    rotatePID.setReference(WristConstants.angledPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stopWrist() {
    rotateSparkMax.set(0);
  }

  public void turnWheel() {
    wheelSparkMax.set(WristConstants.turnWheelPercent);
  }

  public void reverseWheel() {
    wheelSparkMax.set(WristConstants.reverseWheelPercent);
  }

  public boolean hasSomething() {
    return digitalInput.get();
  }

  @Override
  public void periodic() {
  }
}
