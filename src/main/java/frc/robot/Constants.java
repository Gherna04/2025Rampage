// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (75 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16.6);
  
  

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Elevator
  {

    public static final double kElevatorKp = 5.0; //tune
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;
    public static final double kElevatorKf = 0;

    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double kElevatorkS = 0.02; //tune
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;

    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 20;
    public static final double kElevatorCarriageMass = 11;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(10.25);




    public static int m_leftElevatorID = 13;
    public static boolean m_leftElevatorInvert = true;

    public static int m_rightElevatorID = 14;
    public static boolean m_rightElevatorInvert = false;

    public static int elvatorCurrentLimit = 30;
    public static double ELVATOR_SPEED = 1;

    public static int beamID = 1;
    public static int elevatorLimitSwitchID = 3;

    public static final double L1Setpoint = 10;

  }


    public static final int pivotMotorID    = 15;
    public static final int pivotEncoder    = 4;
    public static final double ARM_POWER_SCALING = 0.1;
    public static final double ARM_P_COEFF = 2.5;
    public static final double ARM_I_COEFF = 0;
    public static final double ARM_D_COEFF = 0;
    public static final double ARM_SCORE_HIGH_POSITION = 0.5522-.02;  
    public static final double ARM_SCORE_MID_POSITION = 0.6137;
    public static final double ARM_HUMAN_PLAYER_POSITION = 0.5522- .02;
    public static final double ARM_INSIDE_ROBOT_POSITION = 0.8168;
    public static final double ARM_ENCODER_COUNT_ERROR = 1.0;
    public static final double MAX_ARM_VELOCITY = 0.5;
    public static final double MAX_ARM_ACCELERATION = MAX_ARM_VELOCITY * 2;
    public static final double ARM_ENCODER_COUNT_MAX_DIFF = 0.1;
  

  public static class Coral
  {
    public static int m_coralID = 16;
    public static int coralCurrentLimit = 20;
    public static boolean m_coralInvert = true;
    public static double CORAL_INTAKE_SPEED = .5;
    public static double CORAL_OUTTAKE_SPEED = 1;

    public static int coralLimitSwitchID = 0;

  }
  public static class Hang
  {
    public static int m_HangID = 55;
    public static int HangCurrentLimit = 20;
    public static boolean m_HangInvert = false;
    public static double HANG_INTAKE_SPEED = .2;
    public static double HANG_OUTTAKE_SPEED = .2;

  }

  public static class Algae
  {
    public static int m_algaeID = 17;
    public static int algaeCurrentLimit = 30;
    public static boolean m_algaeInvert = false;
    public static double ALGAE_INTAKE_SPEED = 1;
    public static double ALGAE_OUTTAKE_SPEED = .50;

  }

  public static class OTB
  {
    public static int m_rack = 19;
    public static int rachCurrentLimit = 20;
    public static boolean m_rackInvert = false;
    public static double RACK_SPEED = .5;

    public static int m_otb = 18;
    public static int otbCurrentLimmit = 20;
    public static boolean m_otbInevrt = false;
    public static double OTB_SPEED = .5;


  }
  
}
