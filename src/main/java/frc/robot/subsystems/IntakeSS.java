package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.intakeConstants;

public class IntakeSS extends SubsystemBase {

  @SuppressWarnings("unused")
  private final PneumaticHub ph;
  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;
  private final DigitalInput pressureswitch;

  private final SparkMax wheelSparkMax;

  public IntakeSS() {

    ph = new PneumaticHub(intakeConstants.PneumaticHubPort);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, intakeConstants.leftSolenoidFWDPort, intakeConstants.leftSolenoidRVRSPort);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, intakeConstants.rightSolenoidFWDPort, intakeConstants.rightSolenoidRVRSPort);

    wheelSparkMax = new SparkMax(intakeConstants.wheelSparkMaxPort, MotorType.kBrushless);

    pressureswitch = new DigitalInput(intakeConstants.pressureswitchport);

  }

  public void intakeUp() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  public void intakeDown() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  public void stop() {
    leftSolenoid.set(Value.kOff);
    rightSolenoid.set(Value.kOff);
  }

  public void wheelTurn() {
    wheelSparkMax.set(intakeConstants.wheelTurnPercent);
  }

  public void wheelReverse() {
    wheelSparkMax.set(intakeConstants.wheelReversePercent);
  }

  public boolean getIntaked() {
    return pressureswitch.get();
  }

  @Override
  public void periodic() {
  }
}
