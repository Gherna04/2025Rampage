package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class PivotSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
     private final RelativeEncoder armEncoder;
     private final SparkClosedLoopController armPID;
     private final SparkMaxConfig motorConfig;

     private static final double kP = 0.1;
     private static final double kI = 0.0;
     private static final double kD = 0.0;
     private static final double kMinOutput = -1.0;
     private static final double kMaxOutput = 1.0;
     private static final double kConversionFactor = 0.5;

     public static final double L1 = 0.0;
     public static final double L2 = 20.0;
     public static final double L3 = 40.0;
     public static final double L4 = 60.0;

    public PivotSubsystem() {
        armMotor = new SparkMax(15, MotorType.kBrushless); 
       
        // Get encoder from motor 1
         armEncoder = armMotor.getEncoder();
        

         SparkMaxConfig armconfig = new SparkMaxConfig();
        motorConfig = new SparkMaxConfig();
        armconfig.idleMode(IdleMode.kBrake);

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
         armMotor.configure(armconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

         // Get PID controller
          armPID = armMotor.getClosedLoopController();

        // Reset encoder to 0 at startup
         armEncoder.setPosition(0.0);
    }
    
    
    public Command moveArmUp (double speed){
            return run(()-> {
    
                armMotor.set( -1 * Math.abs(speed));
            
            });

        }

        public Command moveArmDown (double speed){
            return run(()-> {
    
                armMotor.set(  Math.abs(speed));
            
            });

        }


        public Command stopArm (){
            return run(()-> {
    
                armMotor.set( 0);
            
            });

        }
    @Override
    public void periodic() {
        // Code runs periodically
    }
     public void setPosition(double rotations) {
         armPID.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
     }

     public double getPosition() {
         return armEncoder.getPosition();
     }

     public boolean atSetpoint(double target, double tolerance) {
         return Math.abs(getPosition() - target) <= tolerance;
     }



    }
    

