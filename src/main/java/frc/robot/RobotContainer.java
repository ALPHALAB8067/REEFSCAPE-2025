// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import frc.robot.commands.ButtonTest;
import frc.robot.commands.MoveIn3Step;
import frc.robot.commands.MoveIn3StepSecondPosition;
import frc.robot.commands.PID_ExtensionCMD;
import frc.robot.commandgroups.getCoralCMD;
import frc.robot.commands.All_In_One_CMD;
import frc.robot.commands.PID_RotationCMD;
import frc.robot.commands.WristExitCMD;
import frc.robot.commands.WristGoToAngledCMD;
import frc.robot.commands.WristGoToStraightCMD;
import frc.robot.commands.WristIntakeCMD;
import frc.robot.commands.intakeDownCMD;
import frc.robot.commands.intakeUpCMD;
import frc.robot.commands.intakeWheelCMD;
import frc.robot.commands.intakeWheelReverseCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.IntakeSS;
import frc.robot.subsystems.WristSS;
=======
import frc.robot.commands.Autos;
import frc.robot.commands.ButtonTest;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveIn3Step;
import frc.robot.commands.MoveIn3StepSecondPosition;
import frc.robot.commands.PID_ExtensionCMD;
import frc.robot.commands.All_In_One_CMD;
import frc.robot.commands.PID_RotationCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ExampleSubsystem;
>>>>>>> Antoine
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
<<<<<<< HEAD
=======
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
>>>>>>> Antoine

  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         GenericHID mGenericHID = new GenericHID(2);
  Trigger btn1 = new Trigger(()->mGenericHID.getRawButton(1));
  Trigger btn2 = new Trigger(()->mGenericHID.getRawButton(2));
<<<<<<< HEAD
  Trigger btn3 = new Trigger(()->mGenericHID.getRawButton(3));
  Trigger btn4 = new Trigger(()->mGenericHID.getRawButton(4));
  Trigger btn5 = new Trigger(()->mGenericHID.getRawButton(5));
  Trigger btn6 = new Trigger(()->mGenericHID.getRawButton(6));
  Trigger btn17 = new Trigger(()->mGenericHID.getRawButton(17));
  Trigger btn18 = new Trigger(()->mGenericHID.getRawButton(18));
  Trigger btn19 = new Trigger(()->mGenericHID.getRawButton(19));
  Trigger btn20 = new Trigger(()->mGenericHID.getRawButton(20));
  Trigger btn21 = new Trigger(()->mGenericHID.getRawButton(21));
  Trigger btn22 = new Trigger(()->mGenericHID.getRawButton(22));
=======
>>>>>>> Antoine

  private final ARM_SS mArm_SS = new ARM_SS();
  private final PID_ExtensionCMD mPID_ExtensionCMD = new PID_ExtensionCMD(mArm_SS);
  private final PID_RotationCMD mPID_RotationCMD = new PID_RotationCMD(mArm_SS);
  private final All_In_One_CMD mAll_In_One = new All_In_One_CMD(mArm_SS);
  private final MoveIn3Step mMoveIn3Step = new MoveIn3Step(mArm_SS);
  private final MoveIn3StepSecondPosition mMoveIn3StepSecondPosition = new MoveIn3StepSecondPosition(mArm_SS);

  private final PositionsDictionnary mPositionsDictionnary = new PositionsDictionnary();
  private final ButtonTest mButtonTest = new ButtonTest();


<<<<<<< HEAD
  private final IntakeSS mIntakeSS = new IntakeSS();
  private final WristSS mWristSS = new WristSS();
  private final intakeDownCMD mIntakeDownCMD = new intakeDownCMD(mIntakeSS);
  private final intakeUpCMD mIntakeUpCMD = new intakeUpCMD(mIntakeSS);
  private final intakeWheelCMD mIntakeWheelCMD = new intakeWheelCMD(mIntakeSS);
  private final intakeWheelReverseCMD mIntakeWheelReverseCMD = new intakeWheelReverseCMD(mIntakeSS);
  private final WristGoToAngledCMD mWristGoToAngledCMD = new WristGoToAngledCMD(mWristSS);
  private final WristGoToStraightCMD mWristGoToStraightCMD = new WristGoToStraightCMD(mWristSS);
  private final WristExitCMD mWristExitCMD = new WristExitCMD(mWristSS);
  private final WristIntakeCMD mWristIntakeCMD = new WristIntakeCMD(mWristSS);
  private final getCoralCMD mGetCoralCMD = new getCoralCMD(mIntakeSS, mWristSS);
  
  



=======
>>>>>>> Antoine
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
<<<<<<< HEAD
    
=======
>>>>>>> Antoine
    driverXbox.b().whileTrue(mPID_ExtensionCMD);
    driverXbox.a().whileTrue(mPID_RotationCMD);
    driverXbox.y().whileTrue(mAll_In_One);
    driverXbox.x().whileTrue(mMoveIn3Step);
    btn1.whileTrue(mMoveIn3Step);
    btn2.whileTrue(mMoveIn3StepSecondPosition);
    
<<<<<<< HEAD

    //buttonbox version
    btn3.onTrue(mIntakeDownCMD);
    btn4.onTrue(mIntakeUpCMD);
    btn5.onTrue(mIntakeWheelCMD);
    
    btn17.onTrue(mWristGoToAngledCMD);
    btn18.onTrue(mWristGoToStraightCMD);
    btn19.onTrue(mWristIntakeCMD);
    btn20.onTrue(mWristExitCMD);
    btn21.onTrue(mGetCoralCMD);


    //joystick version
    driverXbox.leftBumper().onTrue(mIntakeDownCMD);
    driverXbox.rightBumper().onTrue(mIntakeUpCMD);
    driverXbox.leftTrigger(0.2).onTrue(mIntakeWheelCMD);

    driverXbox.povDown().onTrue(mWristGoToAngledCMD);
    driverXbox.povRight().onTrue(mWristGoToStraightCMD);
    driverXbox.povLeft().onTrue(mWristIntakeCMD);
    driverXbox.povUp().onTrue(mWristExitCMD);
    driverXbox.start().onTrue(mGetCoralCMD);
    


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

=======
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

>>>>>>> Antoine
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
<<<<<<< HEAD
    return null;
=======
    return Autos.exampleAuto(m_exampleSubsystem);
>>>>>>> Antoine
  }
}
