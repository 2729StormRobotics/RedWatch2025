package frc.robot.subsystems.hanger;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.lang.Thread.State;

import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.LED.BlinkinLEDController.BlinkinPattern;
import frc.robot.subsystems.hanger.HangerIOSparkMax.StateMachine;

import com.revrobotics.spark.config.LimitSwitchConfig.Type;

/**
 * Implementation of the HangerIO interface using a Spark Max motor controller.
 */
public class Hanger extends SubsystemBase {
  private final HangerIO io;

  public Hanger(HangerIO io){
    this.io = io;
  }

  @Override
  public void periodic() {
  
  }

}