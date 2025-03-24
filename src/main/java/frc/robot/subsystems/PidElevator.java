
// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;


// public class PidElevator extends SubsystemBase {
//     private final SparkMax primaryMotor;
//     private final SparkMax followerMotor;
//     private final RelativeEncoder encoder;
//     private final DigitalInput bottomLimit;
//     private final PIDController pidController;
//     private final TrapezoidProfile.Constraints constraints;
//     private TrapezoidProfile.State goalState;
//     private TrapezoidProfile.State currentState;
//     private final TrapezoidProfile profile;

//     private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
//     private boolean isHomed = false;
//     private double setpoint = 0.0;
//     SparkMaxConfig resetConfig = new SparkMaxConfig();
//     double currentPos;

//     public enum ElevatorPosition {
//         DOWN(0),
//         POSITION_1(5),
//         POSITION_2(10),
//         POSITION_3(15),
//         POSITION_4(20);

//         public final double positionInches;
        
//         ElevatorPosition(double positionInches) {
//             this.positionInches = positionInches;
//         }
//     }

//     public PidElevator() {
//         primaryMotor = new SparkMax(33, MotorType.kBrushless);
//         followerMotor = new SparkMax(44, MotorType.kBrushless);
        
//         SparkMaxConfig followerConfig = new SparkMaxConfig();
//         followerConfig.follow(primaryMotor, false);

//         // Configure follower
//         followerMotor.configure(followerConfig, null, null); 
        
//         encoder = primaryMotor.getEncoder();
//         bottomLimit = new DigitalInput(Constants.Elevator.beamID);

//         resetConfig.idleMode(IdleMode.kBrake);
//         resetConfig.smartCurrentLimit(40);
//         resetConfig.voltageCompensation(12.0);

//         constraints = new TrapezoidProfile.Constraints(
//             Constants.Elevator.kMaxVelocity,
//             Constants.Elevator.kMaxAcceleration
//         );
        
//         pidController = new PIDController(
//             Constants.Elevator.kElevatorKp,
//             Constants.Elevator.kElevatorKi,
//             Constants.Elevator.kElevatorKd
//         );
        
//         pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
//         // Initialize states and profile
//         currentState = new TrapezoidProfile.State(0, 0);
//         goalState = new TrapezoidProfile.State(0, 0);
//         profile = new TrapezoidProfile(constraints);
        
//         configureMotors();
//     }

//     private void configureMotors() {
//         // Primary motor configuration
//         primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        
//         // Follower motor configuration
//         followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
//     }

//     @Override
//     public void periodic() {

//         currentPos = encoder.getPosition() / 42;
        
//         // Calculate the next state and update current state
//         currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

//         if (bottomLimit.get()) {
//             handleBottomLimit();
//         }

//         if (getHeightInches() > 30) { //maxPos
//             stopMotors();
//         }

//         // Only run control if homed
//         if (isHomed) {
//             double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
//             double ff = calculateFeedForward(currentState);
            
//             double outputPower = MathUtil.clamp(
//                 pidOutput + ff,
//                 -.5, //max output
//                 .5 //max ouput
//             );
            
//             primaryMotor.set(outputPower);
//         }

//         // Update SmartDashboard
//         updateTelemetry();
//     }

//     private void handleBottomLimit() {
//         stopMotors();
//         encoder.setPosition(0 * 42); //bottompos * counts per inch
//         isHomed = true;
//         setpoint = 0; //bottom pos
//         currentState = new TrapezoidProfile.State(0, 0);//bottom pos 
//         goalState = new TrapezoidProfile.State(0, 0); //bottom pos 
//         pidController.reset();
//     }

//     public void stopMotors() {
//         primaryMotor.set(0);
//         pidController.reset();
//     }

//     public boolean isAtHeight(double targetHeightInches) {
//         // Check if the elevator is within a small tolerance of the target height
//         return pidController.atSetpoint() && 
//                Math.abs(getHeightInches() - targetHeightInches) < .5;
//     }
    

//     private double calculateFeedForward(TrapezoidProfile.State state) {
//         // kS (static friction), kG (gravity), kV (velocity),
//         return Constants.Elevator.kElevatorkS * Math.signum(state.velocity) +
//         Constants.Elevator.kElevatorkG +
//         Constants.Elevator.kElevatorkV * state.velocity;
//     }

//     public void setPositionInches(double inches) {
//         if (!isHomed && inches > 0) {
//             System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
//             return;
//         }

//         setpoint = MathUtil.clamp(
//             inches,
//             0, //min pos
//             30 //max pos
//         );
        
//         // Update goal state for motion profile
//         goalState = new TrapezoidProfile.State(setpoint, 0);
//     }

//     private void updateTelemetry() {
//         SmartDashboard.putNumber("Elevator Height", getHeightInches());
//         SmartDashboard.putNumber("Elevator Target", setpoint);
//         SmartDashboard.putBoolean("Elevator Homed", isHomed);
//         SmartDashboard.putString("Elevator State", currentTarget.toString());
//         SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
//         SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
//     }

//     public double getHeightInches() {
//         return encoder.getPosition() / 42; //counts per inch
//     }

//     public void homeElevator() {
//         primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
//         if (bottomLimit.get()) {
//             handleBottomLimit();
//         }
//     }

//     public boolean isAtPosition(ElevatorPosition position) {
//         return pidController.atSetpoint() && 
//                Math.abs(getHeightInches() - position.positionInches) < 0.5;
//     }

//     public boolean isHomed() {
//         return isHomed;
//     }

//     public ElevatorPosition getCurrentTarget() {
//         return currentTarget;
//     }

//     public void setManualPower(double power) {
//         // Disable PID control when in manual mode
//         pidController.reset();
//         currentState = new TrapezoidProfile.State(getHeightInches(), 0);
//         goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
//         if (!isHomed && power < 0) {
//             power = 0;
//         }
        
//         if (getHeightInches() >= 30 && power > 0) { //max pos
//             power = 0;
//         }
        
//         if (bottomLimit.get() && power < 0) {
//             power = 0;
//         }
        
//         primaryMotor.set(MathUtil.clamp(power, -.5, .5));
//     }
// }