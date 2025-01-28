package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.time.Instant;

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
    armSparkMaxConfig.closedLoop.pid(0, 0, 0);
    armSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
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
    public double getVoltage(){
        return armSparkMax.getBusVoltage();
    }

    @Override
    public double getArmAngleRad(){
        return armSparkMax.getAbsoluteEncoder().getPosition();
        //this is not right
    }

    @Override
    public double getArmAngleDegrees(){
        return armSparkMax.getAbsoluteEncoder().getPosition() * (180/Math.PI);
        //this is not righth
    }

    @Override
    public double getArmVelocity(){
        return armSparkMax.getAbsoluteEncoder().getVelocity();
    }

    @Override
    public void pivotCounterclockwise(){
        armSparkMax.set(-ArmIOConstants.kArmMotorSpeed);
    };

    public void goAngle(double kAngle){
        armSparkMax.getClosedLoopController().setReference(kAngle, ControlType.kPosition);
    }
    @Override
    public void stopArm() {
        armSparkMax.stopMotor();
    };

    @Override
    public SequentialCommandGroup clockwise(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                this.clockwise();
            }));
        };

    @Override
    public SequentialCommandGroup counterClockwise(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                this.counterClockwise();
            }));
        };

    @Override
    public SequentialCommandGroup stop(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                this.stop();
            }));
        };

    public SequentialCommandGroup armPosition(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                this.armPosition();
            }));
        };

}