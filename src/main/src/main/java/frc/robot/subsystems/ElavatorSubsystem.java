package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElavatorSubsystem extends SubsystemBase {

 
    // private final RelativeEncoder elavatorEncoder;
    // private final SparkClosedLoopController elavatorPID;


    // private static final double kP = 0.1;
    // private static final double kI = 0.0;
    // private static final double kD = 0.0;
    // private static final double kMinOutput = -1.0;
    // private static final double kMaxOutput = 1.0;
    // private static final double kConversionFactor = 0.5;

    // public static final double L1 = 0.0;
    // public static final double L2 = 20.0;
    // public static final double L3 = 40.0;
    // public static final double L4 = 60.0;

     public static final double speed = 0.5;
     DigitalInput beam = new DigitalInput(8);
     DigitalInput limit = new DigitalInput(3);

     final SparkMax  elavatorMotor1 = new SparkMax(13, MotorType.kBrushless);
     final SparkMax elavatorMotor2 = new SparkMax(14, MotorType.kBrushless);

    public ElavatorSubsystem() {
        // Initialize motors


        SparkMaxConfig leftEmConfig = new SparkMaxConfig();
        leftEmConfig.idleMode(IdleMode.kBrake);
        leftEmConfig.smartCurrentLimit(Constants.Elevator.elvatorCurrentLimit);
        leftEmConfig.inverted(Constants.Elevator.m_leftElevatorInvert);

        elavatorMotor1.configure(leftEmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        SparkMaxConfig rightEmConfig = new SparkMaxConfig();
        rightEmConfig.idleMode(IdleMode.kBrake);
        rightEmConfig.smartCurrentLimit(Constants.Elevator.elvatorCurrentLimit);
        rightEmConfig.inverted(Constants.Elevator.m_rightElevatorInvert);

        elavatorMotor2.configure(rightEmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    



    

        // Get encoder from motor 1
        // elavatorEncoder = elavatorMotor1.getEncoder();

        

        // // Configure encoder conversion factors inside motorConfig
        // motorConfig.encoder
        //     .positionConversionFactor(kConversionFactor)
        //     .velocityConversionFactor(kConversionFactor);

        // // Configure the closed-loop PID controller
        //  motorConfig.closedLoop
        //      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)  // Use internal encoder
        //      .p(kP, ClosedLoopSlot.kSlot0)  // Set PID for position (slot 0)
        //      .i(kI, ClosedLoopSlot.kSlot0)
        //      .d(kD, ClosedLoopSlot.kSlot0)
        //      .outputRange(kMinOutput, kMaxOutput, ClosedLoopSlot.kSlot0);  // Set output range

        // // Apply configuration to Spark MAX
        // elavatorMotor1.configure(em1config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // elavatorMotor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // // Get PID controller
        //  elavatorPID = elavatorMotor1.getClosedLoopController();

        // // Reset encoder to 0 at startup
        // elavatorEncoder.setPosition(0.0);
    }

    // public void moveElevatorUp(double speed) {
    //     elavatorMotor1.set(speed);
    //     elavatorMotor2.set(speed);
    // }

    public Command elvatorUp (double speed){
        return run(()-> {
            elavatorMotor1.set(-1 * Math.abs(speed));
            elavatorMotor2.set(-1 * Math.abs(speed));

        });
    }

    public Command elvatorDown (double speed){
            return run(()-> {
    
                elavatorMotor1.set(Math.abs(speed));
                elavatorMotor2.set(Math.abs(speed));
            });

        }


    public Command elvatorSTOP (double speed){
        return run(()-> {

            elavatorMotor1.set(0);
            elavatorMotor2.set(0);
            
        });

    }

    public boolean getelevatorLimit (){
        return !limit.get();
    }

    public boolean getelevatorBeam (){
        return !beam.get();
    }


}



    // }
    // public void moveElevatordown(double speed) {
    //     elavatorMotor1.set(-speed);
    //     elavatorMotor2.set(-speed);
    // }

    // public void stopElevator() {
    //     elavatorMotor1.set(0.0);
    //     elavatorMotor2.set(0.0);
    // }
//}

    // public void setPosition(double rotations) {
    //     elavatorPID.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // }

    // public double getPosition() {
    //     return elavatorEncoder.getPosition();
    // }

    // public boolean atSetpoint(double target, double tolerance) {
    //     return Math.abs(getPosition() - target) <= tolerance;
    // }
