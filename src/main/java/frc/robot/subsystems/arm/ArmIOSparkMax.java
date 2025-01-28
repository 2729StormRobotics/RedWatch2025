package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;


import frc.robot.subsystems.arm.ArmIOConstants;

public class ArmIOSparkMax implements ArmIO{
    public RelativeEncoder m_ArmRelativeEncoder;
    public SparkMax armSparkMax;
    public SparkMaxConfig armSparkMaxConfig;

    
    public void ArmIOSparkMax() {
    // Define motor
    armSparkMax = new SparkMax(ArmIOConstants.kArmCANID, MotorType.kBrushless);

    m_ArmRelativeEncoder = armSparkMax.getEncoder();

    // Define Configs for Hanger Motor

    armSparkMaxConfig = new SparkMaxConfig();
    armSparkMaxConfig.idleMode(IdleMode.kBrake);

    // burn motor
   armSparkMax.configure(armSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   SparkLimitSwitch metalDetector = armSparkMax.getForwardLimitSwitch();
  }

    

    @Override
    public void pivotClockwise(){
        armSparkMax.set(ArmIOConstants.kArmMotorSpeed);
    };

    @Override
    public void pivotCounterclockwise(){
        armSparkMax.set(-ArmIOConstants.kArmMotorSpeed);
    };

    @Override
    public void stopArm() {
        armSparkMax.stopMotor();
    };

    @Override
    public void setArmPosition(){

    }

}