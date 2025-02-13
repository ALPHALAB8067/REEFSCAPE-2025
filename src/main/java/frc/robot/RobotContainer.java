// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveIn3Step;
import frc.robot.commands.PID_ExtensionCMD;
import frc.robot.commands.All_In_One_CMD;
import frc.robot.commands.PID_RotationCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.PositionsDictionnary;
import frc.robot.subsystems.ExampleSubsystem;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ARM_SS mArm_SS = new ARM_SS();
  private final PID_ExtensionCMD mPID_ExtensionCMD = new PID_ExtensionCMD(mArm_SS);
  private final PID_RotationCMD mPID_RotationCMD = new PID_RotationCMD(mArm_SS);
  private final All_In_One_CMD mAll_In_One = new All_In_One_CMD(mArm_SS);
  private final MoveIn3Step mMoveIn3Step = new MoveIn3Step(mArm_SS);
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  private final PositionsDictionnary mPositionsDictionnary = new PositionsDictionnary();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
    mPositionsDictionnary.initializeTrajectories(mPositionsDictionnary);

    
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
    driverXbox.b().whileTrue(mPID_ExtensionCMD);
    driverXbox.a().whileTrue(mPID_RotationCMD);
    driverXbox.y().whileTrue(mAll_In_One);
    driverXbox.x().whileTrue(mMoveIn3Step);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

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
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
