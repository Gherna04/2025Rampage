
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.invoke.MethodHandles;
import swervelib.SwerveInputStream;


public class RobotContainer
{

    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();


    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    
  // Change to commandJoystick or PS5
  final         CommandPS5Controller driverPS5 = new CommandPS5Controller(0);
  // The robot's subsystems and commands are defined here
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/JsonConstants"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverPS5.getLeftY() * -1,
                                                                () -> driverPS5.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverPS5::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverPS5::getRightX,
                                                                                             driverPS5::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverPS5.getLeftY(),
                                                                        () -> -driverPS5.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverPS5.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math 
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverPS5.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverPS5.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }


  private void configureBindings()
  {




    if (Robot.isSimulation())
    {
      driverPS5.options().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverPS5.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      

      driverPS5.triangle().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverPS5.square().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverPS5.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverPS5.PS().whileTrue(drivebase.centerModulesCommand());
      driverPS5.L1().onTrue(Commands.none());
      driverPS5.R2().onTrue(Commands.none());
    } else
    {
      driverPS5.circle().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverPS5.triangle().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverPS5.cross().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverPS5.options().whileTrue(Commands.none());
      driverPS5.PS().whileTrue(Commands.none());
      driverPS5.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverPS5.R1().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}