package frc.robot.subsystems.hanger;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.lang.Thread.State;

import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.LED.BlinkinLEDController.BlinkinPattern;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

/**
 * Implementation of the HangerIO interface using a Spark Max motor controller.
 */
public class HangerIOSparkMax implements HangerIO {

  private SparkMax hangerSparkMax; // Spark Max motor controller for the hanger
  private SparkMaxConfig hangerConfig; // Configuration for the Spark Max
  public static boolean isClosed = false; // Flag to indicate if the hanger is closed
  public static Timer timer = new Timer(); // Timer for timing operations
  public DigitalInput metalDetector; // Digital input for metal detector
  private BlinkinLEDController ledController = BlinkinLEDController.getInstance(); // LED controller instance

  public enum StateMachine 
  {
    Docked,
    Extending,
    Retracting
  };

  public enum TransitionEvents
  {
    RetractButton,
    ExtendButton,
    None,
  };

  private StateMachine currState = StateMachine.Docked;
  private TransitionEvents currEvent = TransitionEvents.None;

  /**j
   * Constructor for HangerIOSparkMax.
   */
  public HangerIOSparkMax() {
    // Define motor
    hangerSparkMax = new SparkMax(HangerConstants.kWinchMotorCanId, MotorType.kBrushless);

    // Define Configs for Hanger Motor
    hangerConfig = new SparkMaxConfig();
    hangerConfig.idleMode(IdleMode.kBrake); // Set idle mode to brake
    hangerConfig.limitSwitch.forwardLimitSwitchEnabled(true).forwardLimitSwitchType(Type.kNormallyOpen); // Enable forward limit switch
    hangerConfig.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen); // Enable reverse limit switch

    // burn motor
    hangerSparkMax.configure(hangerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // Apply configuration

    metalDetector = new DigitalInput(2); // Initialize metal detector input
  }

  @Override
  public StateMachine getState(){
    return currState;
  }

  @Override
  public void setEvent(TransitionEvents event){
    currEvent = event;
  }

  @Override
  public void setMotorRetract(){
    hangerSparkMax.set(HangerConstants.motorSpeed);
  }

  @Override
  public void setMotorExtend(){
    hangerSparkMax.set(-HangerConstants.motorSpeed);
  }

  @Override
  public void stop(){
    hangerSparkMax.set(0);
  }

  

  /**
   * Sets the voltage of the hanger motor.
   *
   * @param volts The voltage to set.
   */


  /**
   * Periodic method for HangerIOSparkMax.
   * Displays the metal detector state on the SmartDashboard.
   */
}