package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Coral;

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




public class CoralSubsystem extends SubsystemBase {
    
    final SparkMax m_coral = new SparkMax(Constants.Coral.m_coralID, MotorType.kBrushless);
    DigitalInput coral = new DigitalInput(Constants.Coral.coralLimitSwitchID);

    
    {
    
    SparkMaxConfig coralConfig = new SparkMaxConfig();
        coralConfig.idleMode(IdleMode.kCoast);
        coralConfig.smartCurrentLimit(Constants.Coral.coralCurrentLimit);
        coralConfig.inverted(Constants.Coral.m_coralInvert);

        m_coral.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


}
    public Command IntakeCoral (){
        return run(()-> {

            m_coral.set(Constants.Coral.CORAL_INTAKE_SPEED);

        });

    }

    public Command OuttakeCoral (){
        return run(()-> {

            m_coral.set(-1 * Constants.Coral.CORAL_OUTTAKE_SPEED);

        });

    }


    public Command StopCoral (){
    return run(()-> {

        m_coral.set(0);
    });

    }


    public boolean getcoralLimit (){
        return !coral.get();
    }
}



