package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface for Gripper I/O implementations.
 */
public interface GripperIO {

    /**
     * Checks if a coral (or object) is present in the gripper.
     *
     * @return True if coral is present, false otherwise.
     */
    public default boolean isCoralPresent() {
        return false;
    }

    /**
     * Gets the voltage of the gripper motor.
     *
     * @return The voltage.
     */
    public default double getVoltage() {
        return 0.0;
    }

    /**
     * Sets the motor to intake speed.
     */
    public default void setMotorIn() {
    }

    /**
     * Sets the motor to outtake speed.
     */
    public default void setMotorOut() {
    }

    /**
     * Reverses the motor direction.
     */
    public default void reverse() {
    }

    /**
     * Creates a command to intake with the gripper.
     *
     * @return The intake command.
     */
    public default Command Intake() {
        return null;
    }

    /**
     * Creates a command to outtake with the gripper.
     *
     * @return The outtake command.
     */
    public default Command outtake() {
        return null;
    }

    /**
     * Stops the gripper motor.
     */
    public default void stop() {
    }

    /**
     * Auto-logged inputs for the Gripper subsystem.
     */
    @AutoLog
    public static class GripperIOInputs {
        public double gripperAppliedVolts = 0.0; // Applied voltage to the gripper motor
        public double gripperPositionRad = 0.0; // Gripper position in radians
        public double gripperPositionDegrees = 0.0; // Gripper position in degrees
        public double gripperVelocityRadPerSec = 0.0; // Gripper velocity in radians per second
    }

    /**
     * Updates the GripperIOInputs with current sensor data.
     *
     * @param inputs The GripperIOInputs object to update.
     */
    public default void updateInputs(GripperIOInputs inputs) {
    }

    /**
     * Sets the motor speed directly.
     * @param speed the speed to set the motor to.
     */
    public default void setMotorSpeed(double speed) {
    }

    /**
     * Gets the motor encoder position.
     * @return the encoder position.
     */
    public default double getMotorPos() {
        return 0;
    }
}