package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElavatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SetArmPositionCommand extends Command {
    private final PivotSubsystem pivotSubsystem;
    private final double targetPosition;
    private final double tolerance;

    public SetArmPositionCommand(PivotSubsystem subsystem, double targetPosition, double tolerance) {
        this.pivotSubsystem = subsystem;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.setPosition(targetPosition);
    }

     @Override
     public boolean isFinished() {
        return pivotSubsystem.atSetpoint(targetPosition, tolerance);
     }
     }