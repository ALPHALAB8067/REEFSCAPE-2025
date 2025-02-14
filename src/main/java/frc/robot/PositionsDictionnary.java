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
public final class PositionsDictionnary { 
  public static List<PositionType_SS> mTrajectory1;
  public static List<PositionType_SS> mTrajectory2;

 public PositionsDictionnary() {
 PositionType_SS mPosition1 = new PositionType_SS(0.0, 0.0, 2.0);
 PositionType_SS mPosition2 = new PositionType_SS(40.0, 0.0, 2.0);
 PositionType_SS mPosition3 = new PositionType_SS(80.0, 0.0, 2.0);
 mTrajectory1 = new ArrayList<>();
 mTrajectory2 = new ArrayList<>();


  mTrajectory1.add(mPosition1);
  mTrajectory1.add(mPosition2);
  mTrajectory1.add(mPosition3);

  mTrajectory2.add(mPosition3);
  mTrajectory2.add(mPosition2);
  mTrajectory2.add(mPosition1);
}

}