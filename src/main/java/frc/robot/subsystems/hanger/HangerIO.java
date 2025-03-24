package frc.robot.subsystems.hanger;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Interface for Hanger I/O implementations.
 */
public interface HangerIO {

    /**
     * Stops the hanger motor.
     */
    public default void stop() {
    }

    /**
     * Pulls the hanger mechanism.
     */
    public default void pull() {
    }

    /**
     * Releases the hanger mechanism.
     */
    public default void release() {
    }

    /**
     * Sets the voltage of the hanger motor.
     *
     * @param volts The voltage to set.
     */
    public default void setHangerVoltage(double volts) {
    }

    /**
     * Gets the voltage of the hanger motor.
     *
     * @return The voltage.
     */
    public default double getHangerVoltage() {
        return 0;
    }

    /**
     * Gets the angle of the hanger.
     *
     * @return The angle.
     */
    public default double getHangerAngle() {
        return 0;
    }

    /**
     * Gets if the hanger is in the cage.
     *
     * @return True if in the cage, false otherwise.
     */
    public default boolean getIsInCage(){
        return false;
    }

    /**
     * Sets the current limit of the hanger motor.
     *
     * @param limit The current limit.
     */
    public default void setHangerCurrentLimit(int limit) {
    }

    /**
     * Creates a command group to extend the hanger.
     *
     * @return The extend command group.
     */
    public default SequentialCommandGroup extend(){
        return new SequentialCommandGroup(null);
    }

    /**
     * Creates a command group to retract the hanger.
     *
     * @return The retract command group.
     */
    public default SequentialCommandGroup retract(){
        return new SequentialCommandGroup(null);
    }
}