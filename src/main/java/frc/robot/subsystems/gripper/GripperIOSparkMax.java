package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Implementation of the GripperIO interface using a Spark Max motor controller.
 */
public class GripperIOSparkMax implements GripperIO {
    private SparkMax m_gripperMotor; // Spark Max motor controller for the gripper
    private SparkMaxConfig motorConfig; // Configuration for the Spark Max

    /**
     * Constructor for GripperIOSparkMax.
     */
    public GripperIOSparkMax() {
        m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless); // Initialize Spark Max

        // Configure Motor
        motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.pid(GripperConstants.kPGripper, GripperConstants.kIGripper, GripperConstants.kDGripper); // Configure PID
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(false); // Disable forward limit switch
        motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen); // Set limit switch type
        motorConfig.idleMode(IdleMode.kBrake); // Set idle mode to brake
        motorConfig.smartCurrentLimit(40); // Set current limit

        m_gripperMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // Apply configuration

        stop(); // Stop the motor initially
    }

    /**
     * Returns if an object is present in gripper.
     * @return boolean
     */
    @Override
    public boolean isCoralPresent() {
        return m_gripperMotor.getForwardLimitSwitch().isPressed(); // Check forward limit switch
    }

    /**
     * Sets motor speed to 0.
     */
    @Override
    public void stop() {
        m_gripperMotor.set(0); // Stop the motor
    }

    /**
     * Sets motor speed to inwards.
     */
    @Override
    public void setMotorIn() {
        m_gripperMotor.set(GripperConstants.motorSpeedInGripper); // Set motor to intake speed
    }

    /**
     * Sets motor speed to inwards
     */
    @Override
    public void reverse() {
        m_gripperMotor.set(0.6); // Reverse the motor
    }

    /**
     * Sets motor speed to outwards.
     */
    @Override
    public void setMotorOut() {
        m_gripperMotor.set(GripperConstants.motorSpeedOutGripper); // Set motor to outtake speed
    }

    /**
     * Updates the GripperIOInputs with current sensor data.
     * @param inputs GripperIOInputs to update.
     */
    @Override
    public void updateInputs(GripperIOInputs inputs) {
        inputs.gripperAppliedVolts = (m_gripperMotor.getBusVoltage() * m_gripperMotor.getAppliedOutput()); // Update applied voltage
        inputs.gripperPositionDegrees = 0; // Update position (currently 0)
        inputs.gripperVelocityRadPerSec = m_gripperMotor.getEncoder().getVelocity(); // Update velocity
    }

    /**
     * Sets the motor speed directly.
     * @param speed The speed to set the motor to.
     */
    @Override
    public void setMotorSpeed(double speed) {
        m_gripperMotor.set(speed); // Set motor speed
    }

    /**
     * Gets the motor encoder position.
     * @return The encoder position.
     */
    @Override
    public double getMotorPos() {
        return m_gripperMotor.getEncoder().getPosition(); // Get encoder position
    }
}