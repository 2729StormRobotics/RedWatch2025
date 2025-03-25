package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private GripperIO io; // Instance of the GripperIO interface, allowing for different hardware implementations
    private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged(); // Auto-logged inputs for the gripper
    private double holdPosition = 0; // Member variable to store the desired hold position of the gripper

    /**
     * Constructor for the Gripper subsystem.
     *
     * @param io The GripperIO implementation to use.
     */
    public Gripper(GripperIO io) {
        this.io = io; // Assign the provided GripperIO implementation
        SmartDashboard.putData(getName(), this); // Add the subsystem to the SmartDashboard for monitoring
    }

    /**
     * Periodic method called repeatedly to update inputs and log data.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update the inputs from the GripperIO
        Logger.recordOutput("Gripper Position", io.getMotorPos()); // Log the gripper position
        Logger.recordOutput("Gripper Velocity", inputs.gripperVelocityRadPerSec); // Log the gripper velocity
        Logger.recordOutput("Gripper Current", inputs.gripperAppliedVolts); // Log the gripper current
        SmartDashboard.putNumber("Gripper/Position", io.getMotorPos()); // Display position on SmartDashboard
        SmartDashboard.putNumber("Gripper/Velocity", inputs.gripperVelocityRadPerSec); // Display velocity on SmartDashboard
        SmartDashboard.putNumber("Gripper/Current", inputs.gripperAppliedVolts); // Display current on SmartDashboard
        SmartDashboard.putBoolean("Gripper/IsCoralDetected", io.isCoralPresent()); // Display coral detection status
    }

    /**
     * Gets the voltage from the GripperIO.
     *
     * @param voltage The voltage to retrieve.
     * @return The retrieved voltage.
     */
    public double getVoltage(double voltage) {
        return io.getVoltage();
    }

    /**
     * Creates a command to stop the gripper motor.
     *
     * @return A Command that stops the gripper motor.
     */
    public Command stop() {
        return new FunctionalCommand(
                () -> {
                }, // Initialize: No initialization needed
                () -> io.stop(), // Execute: Stop the motor
                (stop) -> io.stop(), // End: Stop the motor
                () -> false, // Is finished: Never finish, run until interrupted
                this); // Pass the current subsystem instance
    }

    /**
     * Creates a command to automatically intake with the gripper.
     *
     * @return A Command that performs automatic intake.
     */
    public Command AutoIntake() {
        return new FunctionalCommand(
                () -> {
                }, // Initialize: No initialization needed
                () -> io.setMotorIn(), // Execute: Set motor to intake speed
                (stop) -> {
                    io.stop(); // End: Stop the motor
                },
                () -> false, // Is finished: Never finish, run until interrupted
                this); // Pass the current subsystem instance
    }

    /**
     * Creates a command to automatically outtake with the gripper.
     *
     * @return A Command that performs automatic outtake.
     */
    public Command AutoOuttake() {
        return new FunctionalCommand(
                () -> {
                }, // Initialize: No initialization needed
                () -> io.setMotorOut(), // Execute: Set motor to outtake speed
                (stop) -> {
                    io.stop(); // End: Stop the motor
                },
                () -> false, // Is finished: Never finish, run until interrupted
                this); // Pass the current subsystem instance
    }

    /**
     * Creates a command to intake with the gripper and then reverse if an object is detected.
     *
     * @return A Command that performs intake and reverse.
     */
    public Command Intake() {
        return new FunctionalCommand(
                () -> {
                }, // Initialize: No initialization needed
                () -> io.setMotorIn(), // Execute: Set motor to intake speed
                (stop) -> {
                    io.stop(); // End: Stop the motor
                },
                () -> io.isCoralPresent(), // Is finished: Finish when coral is detected
                this).andThen(holdPositionCommand()); // Then run reverse for 0 seconds
    }

    /**
     * Creates a command to outtake with the gripper.
     *
     * @return A Command that performs outtake.
     */
    public Command outtake() {
        return new FunctionalCommand(
                () -> {
                }, // Initialize: No initialization needed
                () -> io.setMotorOut(), // Execute: Set motor to outtake speed
                (stop) -> {
                    io.stop(); // End: Stop the motor
                },
                () -> !io.isCoralPresent(), // Is finished: Finish when coral is not detected
                this); // Pass the current subsystem instance
    }

    /**
     * Creates a command to reverse the gripper motor.
     *
     * @return A Command that reverses the motor.
     */
    public Command reverse() {
        return new FunctionalCommand(
                () -> {
                }, // Initialize: No initialization needed
                () -> io.reverse(), // Execute: Reverse the motor
                (stop) -> {
                    io.stop(); // End: Stop the motor
                },
                () -> false, // Is finished: Never finish, run until interrupted
                this); // Pass the current subsystem instance
    }

    /**
     * Creates a command that holds the current position of the gripper motor.
     *
     * @return A Command that holds the current position.
     */
    public Command holdPositionCommand() {
        return new FunctionalCommand(
                () -> holdPosition = io.getMotorPos(), // Capture initial position from GripperIO
                () -> io.setMotorSpeed(GripperConstants.kPGripper * (holdPosition - io.getMotorPos())), // Use P control to hold position
                (interrupted) -> io.stop(), // End: Stop the motor
                () -> false, // Is finished: Never finish, run until interrupted
                this); // Pass the current subsystem instance
    }
}