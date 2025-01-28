package frc.robot;



import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;

public class Constants {
    public static final double ROBOT_MASS = 20;
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, 0), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final boolean smartEnable = false;
    public static final double MAX_SPEED = 0.75;




  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 100; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;

  }



}

