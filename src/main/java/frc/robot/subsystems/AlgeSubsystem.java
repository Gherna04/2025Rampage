package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class AlgeSubsystem extends SubsystemBase {

    final SparkMax m_algae = new SparkMax(Constants.Algae.m_algaeID, MotorType.kBrushless);

    public AlgeSubsystem() {
       
        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.idleMode(IdleMode.kCoast);
        algaeConfig.smartCurrentLimit(Constants.Algae.algaeCurrentLimit);
        algaeConfig.inverted(Constants.Algae.m_algaeInvert);

        m_algae.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
    }


        public Command Intake (){
            return run(()-> {
    
                m_algae.set(-1 * Constants.Algae.ALGAE_INTAKE_SPEED);

            });

        }

    public Command Outtake (){
            return run(()-> {
    
                m_algae.set(Constants.Algae.ALGAE_OUTTAKE_SPEED);

            });

        }


    public Command StopAlgae (){
        return run(()-> {

            m_algae.set(0);
        });

    }
}