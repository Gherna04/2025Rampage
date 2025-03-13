// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

// Subsystems and commands
//Alge
import frc.robot.subsystems.AlgeSubsystem;
import frc.robot.commands.AlgeCommand;
//Elavator
import frc.robot.subsystems.ElavatorSubsystem;
import frc.robot.commands.ElavatorCommand;
import frc.robot.commands.SetElavatorPositionCommand;
//Pivot
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.commands.PivotCommand;

public class RobotContainer
{
  // -------------------------------
  // Controllers
  // -------------------------------
  final CommandPS5Controller driverPs5Controller   = new CommandPS5Controller(0);
  final CommandPS5Controller operatorPs5Controller = new CommandPS5Controller(1);
  //final CommandPS5Controller buttonBoard  = new CommandPS5Controller(2);

  // D-pad buttons for operator
  private final POVButton dpadUpButton    = new POVButton(operatorPs5Controller.getHID(), 0);
  private final POVButton dpadRightButton = new POVButton(operatorPs5Controller.getHID(), 90);
  private final POVButton dpadDownButton  = new POVButton(operatorPs5Controller.getHID(), 180);
  //private final POVButton dpadLeftButton  = new POVButton(operatorPS5Controller.getHID(), 270);

  // -------------------------------
  // Subsystems
  // -------------------------------
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve")
  );


  private final ElavatorSubsystem ElavatorSubsystem = new ElavatorSubsystem();
  private final AlgeSubsystem  AlgeSubsystem      = new AlgeSubsystem();
  private final PivotSubsystem   PivotSubsystem       = new PivotSubsystem();

  // -------------------------------
  // Drive commands
  // -------------------------------
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(
      drivebase,
      () -> -MathUtil.applyDeadband(driverPs5Controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverPs5Controller.getLeftX(), OperatorConstants.DEADBAND),
      () -> -MathUtil.applyDeadband(driverPs5Controller.getRightY(), OperatorConstants.RIGHT_X_DEADBAND),
      driverPs5Controller.getHID()::getSquareButtonPressed,
      driverPs5Controller.getHID()::getCircleButtonPressed,
      driverPs5Controller.getHID()::getTriangleButtonPressed,
      driverPs5Controller.getHID()::getCrossButtonPressed
  );

  // Convert driver input to field-relative controls
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverPs5Controller.getLeftY() * -1,
      () -> driverPs5Controller.getLeftX() * -1
      
  )
  .withControllerRotationAxis(driverPs5Controller::getRightY) //Idk why but when set to X it's on the trigger ¯\_(ツ)_/¯
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true)
  .translationHeadingOffset(true)
  .translationHeadingOffset(drivebase.getHeading());

  // Field-relative input stream
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity.copy()
                          .withControllerHeadingAxis(
                              () -> driverPs5Controller.getRightY() * -1,
                              () -> driverPs5Controller.getRightX() * -1
                          )
                          .headingWhile(true);

  // Default drive commands
  Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveSetpointGen                  = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  // Sim-specific
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -driverPs5Controller.getLeftY(),
      () -> -driverPs5Controller.getLeftX()
  )
  .withControllerRotationAxis(() -> driverPs5Controller.getRawAxis(2))
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleSim =
      driveAngularVelocitySim.copy()
                             .withControllerHeadingAxis(
                                 () -> Math.sin(driverPs5Controller.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                 () -> Math.cos(driverPs5Controller.getRawAxis(2) * Math.PI) * (Math.PI * 2)
                             )
                             .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
  Command driveSetpointGenSim             = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  // ----------------------------------
  // Constructor
  // ----------------------------------
  public RobotContainer()
  {
    // Make sure we bind all the buttons
    configureBindings();

    // Zero the gyro to correct alliance if needed
    drivebase.zeroGyro();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("test", Commands.print("yAA HAA HAA"));
  }

  // ----------------------------------
  // Button/POV bindings
  // ----------------------------------
  private void configureBindings()
  {
    // Set the default drive command, depending on whether we are in sim or real
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedDirectAngle
            : driveFieldOrientedDirectAngleSim
    );

    ElavatorSubsystem.setDefaultCommand(new ElavatorCommand(ElavatorSubsystem, .1)); 
 
    //AlgeSubsystem.setDefaultCommand(new AlgeCommand(AlgeSubsystem, .2));

    if (Robot.isSimulation())
    {
      driverPs5Controller.options().onTrue(Commands.runOnce(() -> 
          drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d())))
      );

    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      driverPs5Controller.triangle().whileTrue(
          Commands.runOnce(drivebase::lock, drivebase).repeatedly()
      );
      driverPs5Controller.square().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverPs5Controller.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverPs5Controller.PS().whileTrue(drivebase.centerModulesCommand());
    }
    else
    {
      // ------------------
      // Driver controls
      // ------------------
      driverPs5Controller.circle().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverPs5Controller.triangle().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverPs5Controller.cross().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(6.161, 3.905), Rotation2d.fromDegrees(158.806))
          )
      );
      driverPs5Controller.square().whileTrue(drivebase.aimAtSpeaker(2));

      driverPs5Controller.options().whileTrue(Commands.none());
      driverPs5Controller.PS().whileTrue(Commands.none());
      driverPs5Controller.L1().whileTrue(
          Commands.runOnce(drivebase::lock, drivebase).repeatedly()
      );

      // ------------------
      // Operator controls
      // ------------------

      // Alge Intake/Outtake Thingy
      operatorPs5Controller.circle().whileTrue(new AlgeCommand(AlgeSubsystem, frc.robot.subsystems.AlgeSubsystem.Intake, 1.0));  // IN
      operatorPs5Controller.cross().whileTrue(new AlgeCommand(AlgeSubsystem, frc.robot.subsystems.AlgeSubsystem.Stop, 1));  // OUT?

      // Elevator Setpoints
      dpadUpButton.onTrue(new SetElavatorPositionCommand(ElavatorSubsystem, frc.robot.subsystems.ElavatorSubsystem.L1, 1.0));
      dpadDownButton.onTrue(new SetElavatorPositionCommand(ElavatorSubsystem, frc.robot.subsystems.ElavatorSubsystem.L2, 1.0));
      dpadRightButton.onTrue(new SetElavatorPositionCommand(ElavatorSubsystem, frc.robot.subsystems.ElavatorSubsystem.L3, 1.0));

      // Pivot
      operatorPs5Controller.triangle().whileTrue(new PivotCommand(PivotSubsystem, 0.35));  // Up
      operatorPs5Controller.square().whileTrue(new PivotCommand(PivotSubsystem, -0.35)); // Down
    }
  }


  // ----------------------------------
  // Autonomous
  // ----------------------------------
  //public Command getAutonomousCommand()
  {
    // Example command used for autonomous
    //return drivebase.getAutonomousCommand("");
  }

  // ----------------------------------
  // Extra methods
  // ----------------------------------
  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}