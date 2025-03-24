// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystems.SwerveSubsystem;


import java.io.File;
import java.io.PrintStream;

import swervelib.SwerveInputStream;

// Subsystems and commands
//Alge
import frc.robot.subsystems.AlgeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.OTBSubsystem;
//Elavator
import frc.robot.subsystems.ElavatorSubsystem;
//import frc.robot.commands.ElavatorCommand;
//import frc.robot.commands.SetArmPositionCommand;
//Pivot
import frc.robot.subsystems.PivotSubsystem;


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
  private final POVButton dpadLeftButton  = new POVButton(operatorPs5Controller.getHID(), 270);

  // -------------------------------
  // Subsystems
  // ------------------------------
   private final ElavatorSubsystem Elavator = new ElavatorSubsystem();
   private final AlgeSubsystem  Alge     = new AlgeSubsystem();
   private final PivotSubsystem   Pivot       = new PivotSubsystem();
   private final CoralSubsystem Coral = new CoralSubsystem();
   private final HangSubsystem Hang = new HangSubsystem();
   private final SwerveSubsystem drivebase = new SwerveSubsystem();
   private final OTBSubsystem OTB = new OTBSubsystem();
   //  private final PidElevator elevator = new PidElevator();
   
  

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
   () -> driverPs5Controller.getLeftY() * 1,
   () -> driverPs5Controller.getLeftX() * 1)
   .withControllerRotationAxis(() -> driverPs5Controller.getRawAxis(
     2)*0.75)
 .deadband(OperatorConstants.DEADBAND)
 .scaleTranslation(0.5)
 .allianceRelativeControl(true);

/**
* Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
*/

SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverPs5Controller::getRightX,
driverPs5Controller::getRightY).headingWhile(true);

/**
* Clone's the angular velocity input stream and converts it to a robotRelative input stream.
*/
SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
        .allianceRelativeControl(false);

SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                   () -> -driverPs5Controller.getLeftY(),
                   () -> -driverPs5Controller.getLeftX())
               .withControllerRotationAxis(() -> driverPs5Controller.getRawAxis(
                   2)*0.5)
               .deadband(OperatorConstants.DEADBAND)
               .scaleTranslation(0.8)
               .allianceRelativeControl(false);
// Derive the heading axis with math!
SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                          .withControllerHeadingAxis(() ->
                                                         Math.sin(
                                                          driverPs5Controller.getRawAxis(
                                                                 2) *
                                                             Math.PI) *
                                                         (Math.PI *
                                                          2),
                                                     () ->
                                                         Math.cos(
                                                          driverPs5Controller.getRawAxis(
                                                                 2) *
                                                             Math.PI) *
                                                         (Math.PI *
                                                          2))
                          .headingWhile(true);
  // ----------------------------------
  // Constructor
  // ----------------------------------
  public RobotContainer()
  {
    //Pathplanner commands
    NamedCommands.registerCommand("L4" ,(new SetArmPositionCommand(Pivot, frc.robot.subsystems.PivotSubsystem.L1, 1.0)));



    
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

    Trigger beamDetector = new Trigger(() -> Elavator.getelevatorBeam());
    Trigger limitDetector = new Trigger(() -> Elavator.getelevatorLimit()); 
    Trigger coralDetector = new Trigger(() -> Coral.getcoralLimit());


    // Set the default drive command, depending on whether we are in sim or real
     Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveRobotOrientedAngularVelocity = drivebase.drive(driveAngularVelocity);
    

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }

      // ------------------
      // Operator controls
      // ------------------

    //   driverPs5Controller.button(3).onTrue(new InstantCommand(() -> {
    //   elevator.setPositionInches(10); 
    //   Commands.print("elevator");
    // }));

    // driverPs5Controller.button(2).onTrue(new InstantCommand(() -> {
    //   elevator.setPositionInches(5); 
    //   Commands.print("Elabator");
    // }));


    




      Alge.setDefaultCommand(Alge.StopAlgae());

      operatorPs5Controller.L2()
      .whileTrue(Alge.Outtake())
      .whileFalse(Alge.StopAlgae());
      
      
      operatorPs5Controller.R2()
      .whileTrue(Alge.Intake())
      .whileFalse(Alge.StopAlgae());
    
      operatorPs5Controller.axisGreaterThan(5, .3)
      .whileTrue(Hang.ReverseHang())
      .whileFalse(Hang.StopHang());
      
      
      operatorPs5Controller.axisLessThan(5,-.3)
      .whileTrue(Hang.Hang())
      .whileFalse(Hang.StopHang());
  



      operatorPs5Controller.L1()
      .whileTrue(Coral.IntakeCoral())
      .whileFalse(Coral.StopCoral());

      operatorPs5Controller.R1()
      .whileTrue(Coral.OuttakeCoral())
      .whileFalse(Coral.StopCoral());


      dpadUpButton.onTrue(new SetArmPositionCommand(Pivot, frc.robot.subsystems.PivotSubsystem.L1, 1.0));
      dpadRightButton.onTrue(new SetArmPositionCommand(Pivot, frc.robot.subsystems.PivotSubsystem.L2, 1.0));
      dpadDownButton.onTrue(new SetArmPositionCommand(Pivot, frc.robot.subsystems.PivotSubsystem.L3, 1.0));
      driverPs5Controller.button(4) 
      .onTrue(new SetArmPositionCommand(Pivot, frc.robot.subsystems.PivotSubsystem.L4, 1.0));


      //Elevator 
      operatorPs5Controller.axisGreaterThan(1, .3) 
      .whileTrue(Elavator.elvatorUp(.1));

      Elavator.setDefaultCommand(Elavator.elvatorSTOP(0));

  

    operatorPs5Controller.axisLessThan(1,-.3)
      .whileTrue(Elavator.elvatorUp(1))
      .whileFalse(Elavator.elvatorSTOP(0));
      
      
    operatorPs5Controller.axisGreaterThan(1, .3)
      .whileTrue(Elavator.elvatorDown(1))
      .whileFalse(Elavator.elvatorSTOP(0));

      limitDetector.whileTrue(Elavator.elvatorSTOP(0));
      
      beamDetector.whileTrue(Elavator.elvatorSTOP(0));
     // coralDetector.whileTrue(Coral.StopCoral());


    operatorPs5Controller.button(4)
      .whileTrue(Pivot.moveArmDown(-1))
      .whileFalse(Pivot.stopArm());
        
      

    operatorPs5Controller.button(2)
      .whileTrue(Pivot.moveArmUp(1))
      .whileFalse(Pivot.stopArm());
    }



  // ----------------------------------
  // Autonomous
  // ----------------------------------
  public Command getAutonomousCommand()
  {

  return new PathPlannerAuto("Prototype Auto");

  }

  // ----------------------------------
  // Extra methods
  // ----------------------------------
  // public void setDriveMode()
  // {
  //   configureBindings();
  // }

  // public void setMotorBrake(boolean brake)
  // {
  //   drivebase.setMotorBrake(brake);
  // } 
}
