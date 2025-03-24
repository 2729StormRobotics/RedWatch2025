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
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.hanger.HangerConstants;
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

  /**
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

  /**
   * Sets the voltage of the hanger motor.
   *
   * @param volts The voltage to set.
   */
  @Override
  public void setHangerVoltage(double volts) {
    hangerSparkMax.setVoltage(volts);
  }

  /**
   * Gets the voltage of the hanger motor.
   *
   * @return The voltage.
   */
  @Override
  public double getHangerVoltage() {
    return hangerSparkMax.getBusVoltage() * hangerSparkMax.getAppliedOutput();
  }

  /**
   * Gets the angle of the hanger.
   *
   * @return The angle (currently 0, needs testing).
   */
  @Override
  public double getHangerAngle() {
    // NEEDS TO BE TESTED
    return 0;
  }

  /**
   * Gets if the hanger is in the cage.
   *
   * @return True if in the cage, false otherwise.
   */
  @Override
  public boolean getIsInCage(){
    return metalDetector.get();
  }

  /**
   * Pulls the hanger mechanism.
   */
  @Override
  public void pull() {
    BlinkinLEDController.isEndgame = true; // Set endgame flag for LED control

    hangerSparkMax.set(HangerConstants.motorSpeedOpenHanger); // Set motor to pull speed
    isClosed = true;
  }

  /**
   * Releases the hanger mechanism.
   */
  @Override
  public void release() {
    BlinkinLEDController.isEndgame = true; // Set endgame flag for LED control

    hangerSparkMax.set(-1); // Set motor to release speed
    ledController.setPattern(BlinkinPattern.RAINBOW_RAINBOW_PALETTE); // Set LED pattern
    isClosed = false;
  }

  /**
   * Stops the hanger motor.
   */
  @Override
  public void stop() {
    hangerSparkMax.set(0); // Stop the motor
  }

  /**
   * Creates a command group to retract the hanger.
   *
   * @return The retract command group.
   */
  @Override
  public SequentialCommandGroup retract() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {System.out.println("retract");}), // Print retract message
        new InstantCommand(() -> {
          this.pull(); // Pull the hanger
        }),
        new WaitCommand(5), // Wait for 5 seconds
        new InstantCommand(() -> {
          this.stop(); // Stop the motor
        }));
  }

  /**
   * Periodic method for HangerIOSparkMax.
   * Displays the metal detector state on the SmartDashboard.
   */
  public void periodic(){
    SmartDashboard.putBoolean("metaldetector", getIsInCage()); // Display metal detector state
  }
}