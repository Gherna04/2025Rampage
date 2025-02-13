// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();


    static
    {
        System.out.println("Loading: " + fullClassName);
    }
  private Main()
  {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args)
  {
    RobotBase.startRobot(Robot::new);
  }
}
