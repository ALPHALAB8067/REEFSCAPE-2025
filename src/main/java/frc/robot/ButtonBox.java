// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** This is a filter class for the custom buttons on the driver station */
public class ButtonBox {
  private Joystick controllerPT1;
  private Joystick controllerPT2;
  private GenericHID mGenericHID;

  /**
   * Filter for the custom buttons and switches
   *
   * @param port the port the controller is connected to on thePT1 driver station
   */
  public ButtonBox(int port1, int port2) {
    this.controllerPT1 = new Joystick(port1);
    this.controllerPT2 = new Joystick(port2);

  }

  /**
   * Returns if the L1 button has been pressed
   *
   * @return boolean
   */
  public boolean getSwitch1() {
    //return controllerPT1.getRawButton(1);
    return controllerPT1.getRawButton(2);
  }

  /**
   * Returns if the L2 button has been pressed
   *
   * @return boolean
   */
  public boolean getSwitch2() {
    return controllerPT1.getRawButton(2);
  }

  /**
   * Returns if the L3 button has been pressed
   *
   * @return boolean
   */
  public boolean getSwitch3() {
    return controllerPT1.getRawButton(3);
  }

  /**
   * Returns if the L4 button has been pressed
   *
   * @return boolean
   */
  public boolean getSwitch4() {
    return controllerPT1.getRawButton(4);
  }

  /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch5() {
    return controllerPT1.getRawButton(5);
  }

  /**
   * returns if the bottom switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch6() {
    return controllerPT1.getRawButton(6);
  }
    /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch7() {
    return controllerPT1.getRawButton(7);
  }
    /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch8() {
    return controllerPT1.getRawButton(8);
  }
    /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch9() {
    return controllerPT1.getRawButton(10);
  }
    /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch10() {
    return controllerPT1.getRawButton(11);
  }
    /**
   * returns if the chute switch is on or off
   *
   * @return boolean
   */
  public boolean getSwitch11() {
    return controllerPT1.getRawButton(12);
  }

}