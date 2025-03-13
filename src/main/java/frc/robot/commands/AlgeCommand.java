package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeSubsystem;

public class AlgeCommand extends Command {
    private final AlgeSubsystem algeSubsystem;
    private final double targetPosition;
    private final double tolerance;

    public AlgeCommand(AlgeSubsystem subsystem, double targetPosition, double tolerance) {
        this.algeSubsystem = subsystem;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        algeSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return algeSubsystem.atSetpoint(targetPosition, tolerance);
    }
}