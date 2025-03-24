package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Coral;
import frc.robot.Constants.Hang;

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




public class HangSubsystem extends SubsystemBase {
    
    final SparkMax m_Hang = new SparkMax(Constants.Hang.m_HangID, MotorType.kBrushless);

    
    {
    
    SparkMaxConfig hangConfig = new SparkMaxConfig();
        hangConfig.idleMode(IdleMode.kCoast);
        hangConfig.smartCurrentLimit(Constants.Hang.HangCurrentLimit);
        hangConfig.inverted(Constants.Hang.m_HangInvert);

        m_Hang.configure(hangConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


}
    public Command Hang (){
        return run(()-> {

            m_Hang.set(Constants.Hang.HANG_INTAKE_SPEED);

        });

    }

    public Command ReverseHang (){
        return run(()-> {

            m_Hang.set(-1* Constants.Hang.HANG_INTAKE_SPEED);

        });

    }


    public Command StopHang (){
    return run(()-> {

        m_Hang.set(0);
    });

    }
}

