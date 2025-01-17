package frc.robot.subsystems.hanger;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.drive.MotorConfigs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Queue;


// public class HangerIOSparkFlex implements HangerIO{

//     private SparkFlex hangerSparkFlex;
//     private SparkFlexConfig hangerConfig;

//     @Override
//   public void setHangerVoltage(double volts) {
//     hangerSparkFlex.setVoltage(volts);
//   } 

//   public double getHangerVoltage() {
//     return hangerSparkFlex.getBusVoltage() * hangerSparkFlex.getAppliedOutput();
//   }
    
// }
