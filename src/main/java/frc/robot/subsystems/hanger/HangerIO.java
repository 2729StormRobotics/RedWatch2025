package frc.robot.subsystems.hanger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.subsystems.hanger.HangerConstants.MAX_VOLTAGE;

import org.littletonrobotics.junction.AutoLog;


public interface HangerIO{


        public static class HangerIOInputs{
    
        }
    
        public void stop();
        
        public void pull();


        public void release();

    //public void updateInputs(HangerIOInputs inputs) {
        
    //}

}
