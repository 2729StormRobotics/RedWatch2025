package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for controlling the arm mechanism.
 * Defines the methods that must be implemented by any class that controls the arm.
 */
public interface ArmIO {

    /**
     * Pivots the arm clockwise.
     */
    public default void pivotClockwise() {
    };

    /**
     * Pivots the arm counterclockwise.
     */
    public default void pivotCounterclockwise() {
    };

    /**
     * Sets the desired position of the arm.
     *
     * @param position The target position for the arm.
     */
    public default void setArmPosition(double position) {
    };

    /**
     * Stops the arm motor.
     */
    public default void stopArm() {
    };

    /**
     * Changes the encoder offset of the arm.
     *
     * @param newOffset The new offset value.
     */
    public default void changeOffset(double newOffset){

    }

    /**
     * Gets the state of the hall effect sensor.
     *
     * @return True if the hall effect sensor is triggered, false otherwise.
     */
    public default boolean getHallEffect(){return false;};

    /**
     * Gets the voltage applied to the arm motor.
     *
     * @return The voltage applied to the arm motor.
     */
    public default double getVoltage() {
        return 0.0;
    };

    /**
     * Gets the current angle of the arm in radians.
     *
     * @return The current angle of the arm in radians.
     */
    public default double getArmAngleRad() {
        return 0.0;
    };

    /**
     * Gets the current angle of the arm in degrees.
     *
     * @return The current angle of the arm in degrees.
     */
    public default double getArmAngleDegrees() {
        return 0.0;
    };

    /**
     * Gets the current velocity of the arm.
     *
     * @return The current velocity of the arm.
     */
    public default double getArmVelocity() {
        return 0.0;
    }; // i forgot the units

    /**
     * Sets the voltage applied to the arm motor.
     *
     * @param voltage The voltage to apply to the arm motor.
     */
    public default void setVoltage(double voltage) {
    };

    /**
     * Sets the speed of the arm motor.
     *
     * @param speed The speed to set the arm motor.
     */
    public default void setSpeed(double speed) {
    };

    /**
     * Sets the P gain for PID control.
     *
     * @param p The P gain value.
     */
    public default void setP(double p) {}

    /**
     * Sets the I gain for PID control.
     *
     * @param i The I gain value.
     */
    public default void setI(double i) {}

    /**
     * Sets the D gain for PID control.
     *
     * @param d The D gain value.
     */
    public default void setD(double d) {}

    /**
     * Sets the feedforward gain for PID control.
     *
     * @param ff The feedforward gain value.
     */
    public default void setFF(double ff) {}

    /**
     * Sets the static friction gain for PID control.
     *
     * @param kS The static friction gain value.
     */
    public default void setkS(double kS) {}

    /**
     * Sets the velocity gain for PID control.
     *
     * @param kV The velocity gain value.
     */
    public default void setkV(double kV) {}

    /**
     * Sets the gravity gain for PID control.
     *
     * @param kG The gravity gain value.
     */
    public default void setkG(double kG) {}

    /**
     * Sets the acceleration gain for PID control.
     *
     * @param kA The acceleration gain value.
     */
    public default void setkA(double kA) {}

    /**
     * Gets the P gain for PID control.
     *
     * @return The P gain value.
     */
    public default double getP() {
        return 0.0;
    }

    /**
     * Gets the I gain for PID control.
     *
     * @return The I gain value.
     */
    public default double getI() {
        return 0.0;
    }

    /**
     * Gets the D gain for PID control.
     *
     * @return The D gain value.
     */
    public default double getD() {
        return 0.0;
    }

    /**
     * Gets the feedforward gain for PID control.
     *
     * @return The feedforward gain value.
     */
    public default double getFF() {
        return 0.0;
    }

    /**
     * Gets the static friction gain for PID control.
     *
     * @return The static friction gain value.
     */
    public default double getkS() {
        return 0.0;
    }

    /**
     * Gets the gravity gain for PID control.
     *
     * @return The gravity gain value.
     */
    public default double getkG() {
        return 0.0;
    }

    /**
     * Gets the velocity gain for PID control.
     *
     * @return The velocity gain value.
     */
    public default double getkV() {
        return 0.0;
    }

    /**
     * Gets the acceleration gain for PID control.
     *
     * @return The acceleration gain value.
     */
    public default double getkA() {
        return 0.0;
    }

    /**
     * Auto-logged inputs for the ArmIO interface.
     */
    @AutoLog
    public static class ArmIOInputs {
        // public ClosedLoopConfig armCLC = new ClosedLoopConfig();
        public double armAppliedVolts = 0.0;
        public double armPositionRad = 0.0;
        public double armPositionDegrees = 0.0;
        public double armVelocityRadPerSec = 0.0;

    }

    /**
     * Updates the ArmIOInputs with the current values.
     *
     * @param inputs The ArmIOInputs to update.
     */
    public default void updateInputs(ArmIOInputs inputs) {}
}