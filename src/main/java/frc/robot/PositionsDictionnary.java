// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.PositionsDictionnary.ArmPositionType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class PositionsDictionnary { 
  
  public static class ArmPositionType {
    public final double angle;
    public final double length;
    public final double threshold;

    public ArmPositionType(double angle, double length, double threshold) {
      this.angle = angle;
      this.length = length;
      this.threshold = threshold;

  } 
  }
 public static class trajectories {
  ArmPositionType mPosition1 = new ArmPositionType(20, 0, 2 );
  ArmPositionType mPosition2 = new ArmPositionType(40, 0, 2 );
  ArmPositionType mPosition3 = new ArmPositionType(80, 0, 2 );
  public static List<ArmPositionType> mTrajectory1 = new ArrayList<>();
  public static List<ArmPositionType> mTrajectory2 = new ArrayList<>();
  {
    mTrajectory1.add(mPosition1);
    mTrajectory1.add(mPosition2);
    mTrajectory1.add(mPosition3);

    mTrajectory2.add(mPosition3);
    mTrajectory2.add(mPosition2);
    mTrajectory2.add(mPosition1);
  }
}
}