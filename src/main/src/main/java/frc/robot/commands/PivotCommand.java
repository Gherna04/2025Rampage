// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.PivotSubsystem;

// public class PivotCommand extends Command {
//     private final PivotSubsystem armSubsystem;
//     private final double speed;

//     public PivotCommand(PivotSubsystem subsystem, double speed) {
//         this.armSubsystem = subsystem;
//         this.speed = speed;
//         addRequirements(subsystem); // Prevents other commands from interrupting
//     }

//     @Override
//     public void execute() {
//         armSubsystem.moveArmUp(speed); // Moves the climbing mechanism at the specified speed
//     }

//     @Override
//     public void end(boolean interrupted) {
//         armSubsystem.stopArm(); // Stops the motor when the command ends
//     }

//     @Override
//     public boolean isFinished() {
//         return false; // Runs continuously until interrupted
//     }
// }