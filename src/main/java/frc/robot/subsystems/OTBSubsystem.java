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

public class OTBSubsystem extends SubsystemBase {

 

     final SparkMax  Rack = new SparkMax(Constants.OTB.m_rack, MotorType.kBrushless);
     final SparkMax OTB = new SparkMax(Constants.OTB.m_otb, MotorType.kBrushless);

    public OTBSubsystem() {
        // Initialize motors


        SparkMaxConfig RackConfig = new SparkMaxConfig();
        RackConfig.idleMode(IdleMode.kBrake);
        RackConfig.smartCurrentLimit(Constants.OTB.rachCurrentLimit);
        RackConfig.inverted(Constants.OTB.m_rackInvert);

        Rack.configure(RackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        SparkMaxConfig otbConfig = new SparkMaxConfig();
        otbConfig.idleMode(IdleMode.kBrake);
        otbConfig.smartCurrentLimit(Constants.OTB.otbCurrentLimmit);
        otbConfig.inverted(Constants.OTB.m_otbInevrt);

        OTB.configure(otbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public Command OTBBack () {
        return run(()-> {
            Rack.set(-1 * Math.abs(Constants.OTB.RACK_SPEED));
            OTB.set(-1 * Math.abs(Constants.OTB.OTB_SPEED));

        });
    }

    public Command OTBFoward (){
            return run(()-> {
    
                Rack.set(Math.abs(Constants.OTB.RACK_SPEED));
                OTB.set(Math.abs(Constants.OTB.OTB_SPEED));
            });

        }

    public Command OTBIntake () {
        return run(() -> {

            OTB.set(Constants.OTB.OTB_SPEED);

        });
    }

    public Command OTBOuttake () {
        return run(() -> {

            OTB.set(-1 * Constants.OTB.OTB_SPEED);

        });
    }


    public Command OTBSTOP (){
        return run(()-> {

            Rack.set(0);
            OTB.set(0);
            
        });

    }



}
