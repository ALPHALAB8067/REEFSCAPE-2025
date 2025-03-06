// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import frc.robot.subsystems.PositionType_SS;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class PositionsDictionnary {                        //(angle bras, angle wrist, longueur bras, tolerance bras, tolerance wrist, tolerance longueur, claw)
  public static PositionType_SS mPositionRest = new PositionType_SS(0,     115,0, 2,       5,     2,  90, "rest");
  public static PositionType_SS mPositionL1 = new PositionType_SS(  45,          -85, 4, 2,5,     2,   0,"L1");
  public static PositionType_SS mPositionL2 = new PositionType_SS(  0,     53, 0, 2,       5,     2,  90,"L2");
  public static PositionType_SS mPositionL3 = new PositionType_SS(  34,    23, 6, 2,       5,     2,  90,"L3");
  public static PositionType_SS mPositionL4 = new PositionType_SS(  70,          -30, 30,2,5,     2,  90,"L4");
  public static PositionType_SS mPositionCoralStation = new PositionType_SS(45, -45,  5, 2,5,     2,   0, "coralStation");
  //public static PositionType_SS mPositionIntake = new PositionType_SS(0.0, 0.0, 2.0,2.0,2.0,2.0,0);
  
}