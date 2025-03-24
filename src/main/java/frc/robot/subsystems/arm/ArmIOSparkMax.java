package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;


public class ArmIOSparkMax implements ArmIO {
    public SparkMax armSparkMaxLeft; // Left Spark Max motor for controlling arm movement
    public SparkMax armSparkMaxRight; // Right Spark Max motor, configured to follow the left motor
    public SparkMaxConfig armConfigRight; // Configuration for the right Spark Max motor
    public SparkMaxConfig armConfigLeft; // Configuration for the left Spark Max motor
    public SparkLimitSwitch hallEffect; // Hall effect sensor to detect a specific arm position
    public AbsoluteEncoder armAbsoluteEncoder; // Absolute encoder for precise arm position feedback
    public double armEncoderOffset; // Offset value for the absolute encoder reading

    private double kP = ArmConstants.kPArm; // Proportional gain for PID control
    private double kI = ArmConstants.kIArm; // Integral gain for PID control
    private double kD = ArmConstants.kDArm; // Derivative gain for PID control

    /*
     * This subsystem uses 2 motors to rotate our pivot arm from 0 - 180 degrees
     * Inputs are given in Degrees
     *
     */
    public ArmIOSparkMax() {
        // Define motor
        armSparkMaxLeft = new SparkMax(ArmConstants.kArmCANID, MotorType.kBrushless); // Initialize left motor
        armSparkMaxRight = new SparkMax(ArmConstants.kArmCANID2, MotorType.kBrushless); // Initialize right motor

        // Define Configs for Hanger Motor
        armConfigRight = new SparkMaxConfig(); // Create configuration object for right motor
        armConfigRight.idleMode(IdleMode.kCoast); // Set to coast mode when no power is applied
        armConfigRight.limitSwitch.reverseLimitSwitchEnabled(false); // Disable reverse limit switch

        armConfigLeft = new SparkMaxConfig(); // Create configuration for left motor.
        armConfigLeft.apply(armConfigRight); // Apply the settings from armConfigRight to armConfigLeft
        armConfigLeft.closedLoop.pid(kP, kI, kD); // Set the PID gains for closed-loop control
        armConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // Use the absolute encoder as the feedback sensor
        armConfigLeft.closedLoop.outputRange(ArmConstants.kArmMinOutputPower,
                ArmConstants.kArmMaxOutputPower); // Set the output range of the motor controller
        armConfigLeft.absoluteEncoder.velocityConversionFactor(6);   // Set factor to convert encoder velocity to meaningful units.
        armConfigLeft.absoluteEncoder.positionConversionFactor(360); // Set factor to convert encoder position to meaningful units (degrees).

        armConfigRight.follow(armSparkMaxLeft, true); // Configure the right motor to follow the left motor, inverted

        // burn motor
        armSparkMaxLeft.configure(armConfigLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters); // Configure left motor and persist settings to flash
        armSparkMaxRight.configure(armConfigRight, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters); // Configure right motor and persist settings
        // armSparkMaxRight.pauseFollowerMode();

        hallEffect = armSparkMaxRight.getReverseLimitSwitch(); // Get the hall effect sensor object from the right motor
        armAbsoluteEncoder = armSparkMaxLeft.getAbsoluteEncoder(); // Get the absolute encoder object from the left motor
    }

    /**
     * Updates the inputs for the arm subsystem.  This method is called periodically
     * by the main robot loop.
     *
     * @param inputs The ArmIOInputs object to update with the latest sensor values.
     */
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armAppliedVolts = (armSparkMaxRight.getBusVoltage()
                * armSparkMaxRight.getAppliedOutput()); // Get the applied voltage to the right motor
        inputs.armPositionDegrees = getArmAngleDegrees(); // Get the current arm position in degrees
        inputs.armVelocityRadPerSec = getArmVelocity(); // Get the current arm velocity in radians per second
    }

    /**
     * Pivots the arm clockwise at a predefined speed.
     */
    @Override
    public void pivotClockwise() {
        armSparkMaxLeft.set(ArmConstants.kArmMotorSpeed); // Set the left motor speed to rotate clockwise
    }

    /**
     * Gets the state of the hall effect sensor.
     *
     * @return true if the hall effect sensor is pressed, false otherwise.
     */
    @Override
    public boolean getHallEffect() {
        return hallEffect.isPressed();
    }

    /**
     * Changes the offset of the absolute encoder.  This is used to calibrate the
     * encoder reading.
     *
     * @param newOffset The new offset value to add to the current offset.
     */
    @Override
    public void changeOffset(double newOffset) {
        armEncoderOffset += newOffset;
    }

    /**
     * Gets the voltage being supplied to the arm motor.
     *
     * @return The bus voltage of the right Spark Max motor.
     */
    @Override
    public double getVoltage() {
        return armSparkMaxRight.getBusVoltage();
    }

    /**
     * Gets the current arm angle in degrees, as measured by the absolute encoder.
     *
     * @return The arm angle in degrees.
     */
    @Override
    public double getArmAngleDegrees() {
        return armAbsoluteEncoder.getPosition();
    }

    /**
     * Gets the current arm velocity in radians per second, as measured by the
     * absolute encoder.
     *
     * @return The arm velocity in radians per second.
     */
    @Override
    public double getArmVelocity() {
        return armAbsoluteEncoder.getVelocity();
    }

    /**
     * Pivots the arm counterclockwise at a predefined speed.
     */
    @Override
    public void pivotCounterclockwise() {
        armSparkMaxLeft.set(-ArmConstants.kArmMotorSpeed); // Set the left motor speed to rotate counterclockwise
    };

    /**
     * Sets the voltage to be applied to the arm motor.
     *
     * @param voltage The voltage to set.
     */
    @Override
    public void setVoltage(double voltage) {
        armSparkMaxLeft.setVoltage(voltage); // Set the voltage of the left motor.  The right motor follows.
    }

    /**
     * Sets the speed of the arm motor.
     *
     * @param speed The speed to set.
     */
    @Override
    public void setSpeed(double speed) {
        armSparkMaxLeft.set(speed); // Set the speed of the left motor. The right motor follows.
    }

    /**
     * Sets the desired arm position using closed-loop control.
     *
     * @param kAngle The target arm angle in degrees.
     */
    @Override
    public void setArmPosition(double kAngle) {
        armSparkMaxLeft.getClosedLoopController().setReference(kAngle, ControlType.kPosition); // Set the position setpoint for the left motor's closed-loop controller.
    }

    /**
     * Stops the arm motor.
     */
    @Override
    public void stopArm() {
        armSparkMaxLeft.stopMotor(); // Stop the left motor.  The right motor, as a follower, will also stop.
    };

    /**
     * Helper method to update the configuration of both arm motors.
     *
     * @param config The SparkMaxConfig object containing the new configuration
     * parameters.
     */
    private void updateMotorConfig(SparkMaxConfig config) {
        // DO NOT RESET parameters because we only want to change some parameters, not all
        // DO NOT PERSIST because this is a temporary change that we don't want to save
        // to memory
        armSparkMaxLeft.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters); // Apply the configuration to the left motor
        armSparkMaxRight.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters); // Apply the configuration to the right motor
    }

    /**
     * Sets the proportional gain (P) for the PID controller.
     *
     * @param kP The new proportional gain value.
     */
    @Override
    public void setP(double kP) {
        this.kP = kP;
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(kP);
        updateMotorConfig(config);
    }

    /**
     * Sets the integral gain (I) for the PID controller.
     *
     * @param kI The new integral gain value.
     */
    @Override
    public void setI(double kI) {
        this.kI = kI;
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.i(kI);
        updateMotorConfig(config);
    }

    /**
     * Sets the derivative gain (D) for the PID controller.
     *
     * @param kD The new derivative gain value.
     */
    @Override
    public void setD(double kD) {
        this.kD = kD;
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.d(kD);
        updateMotorConfig(config);
    }

    /**
     * Gets the proportional gain (P) of the PID controller.
     *
     * @return The current proportional gain value.
     */
    @Override
    public double getP() {
        return kP;
    }

    /**
     * Gets the integral gain (I) of the PID controller.
     *
     * @return The current integral gain value.
     */
    @Override
    public double getI() {
        return kI;
    }

    /**
     * Gets the derivative gain (D) of the PID controller.
     *
     * @return The current derivative gain value.
     */
    @Override
    public double getD() {
        return kD;
    }
}
