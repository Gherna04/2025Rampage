package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class AlgeSubsystem extends SubsystemBase {
    private final SparkMax algeMotor;
    private final RelativeEncoder algeEncoder;
    private final SparkClosedLoopController algePID;
    private final SparkMaxConfig motorConfig;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMinOutput = -1.0;
    private static final double kMaxOutput = 1.0;
    private static final double kConversionFactor = 0.5;

    public static final double Intake = .5;
    public static final double Stop = 0;
    public static final double Outtake = -0.5;

    public AlgeSubsystem() {
        // Initialize motors
        algeMotor = new SparkMax(60, MotorType.kBrushless);

        // Get encoder from motor 1
        algeEncoder = algeMotor.getEncoder();

        // Create motor configuration object
        motorConfig = new SparkMaxConfig();

        // Configure encoder conversion factors inside motorConfig
        motorConfig.encoder
            .positionConversionFactor(kConversionFactor)
            .velocityConversionFactor(kConversionFactor);

        // Configure the closed-loop PID controller
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)  // Use internal encoder
            .p(kP, ClosedLoopSlot.kSlot0)  // Set PID for position (slot 0)
            .i(kI, ClosedLoopSlot.kSlot0)
            .d(kD, ClosedLoopSlot.kSlot0)
            .outputRange(kMinOutput, kMaxOutput, ClosedLoopSlot.kSlot0);  // Set output range

        // Apply configuration to Spark MAX
        algeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get PID controller
        algePID = algeMotor.getClosedLoopController();

        // Reset encoder to 0 at startup
        algeEncoder.setPosition(0.0);
    }

    public void Intake(double speed) {
        algeMotor.set(speed);
    }

    public void stopIntake() {
        algeMotor.set(0.0);
    }

    public void setPosition(double rotations) {
        algePID.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getPosition() {
        return algeEncoder.getPosition();
    }

    public boolean atSetpoint(double target, double tolerance) {
        return Math.abs(getPosition() - target) <= tolerance;
    }
}