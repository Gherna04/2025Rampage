package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax armMotor;

    public PivotSubsystem() {
        armMotor = new SparkMax(56, MotorType.kBrushless); // CAN ID 60
        // Reminder Configure this Motor
    }

    public void moveArm(double speed) {
        armMotor.set(speed);
    }

    public void stopArm() {
        armMotor.set(0);
    }

    @Override
    public void periodic() {
        // Code runs periodically
    }
}