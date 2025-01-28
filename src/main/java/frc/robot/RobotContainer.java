// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;//
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveSubsystem;

import java.io.File;



import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;//
import com.pathplanner.lib.commands.PathPlannerAuto;



public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));


  private final SendableChooser<Command> autoChooser;

  final CommandPS5Controller driverPs5 = new CommandPS5Controller(0);





  

  public RobotContainer() {


   autoChooser = AutoBuilder.buildAutoChooser();
   SmartDashboard.putData("Auto Chooser", autoChooser);



   configureBindings();

    drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverPs5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverPs5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverPs5.getRightX());


    

    //------------------------------CONTROLS---------------------------------------------------------

    



  }

  



  private void configureBindings() {

  }


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Playoffs");
    

  }



  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
