package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Simulation implementation of the GripperIO interface.
 */
public class GripperIOSim implements GripperIO {
    // private SimD m_simObjectDectector; // Simulated object detector (commented out)
    private SparkMax m_simgripperMotor; // Simulated gripper motor

    /**
     * Constructor for GripperIOSim.
     *
     * @param isSimulation Indicates if the robot is in simulation mode.
     */
    public void GripperIO(boolean isSimulation) {
        if (isSimulation) {
            // m_simObjectDectector = new DigitalInput(GripperConstants.beambreakPort); // Simulated object detector (commented out)
            m_simgripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless); // Initialize simulated motor
        }
    }

    /**
     * Checks if a coral (or object) is present in the gripper (simulation).
     *
     * @return Always false in simulation.
     */
    public boolean isCoralPresent() {
        // Have to change for the promxity sensor // Comment indicating need to change for proximity sensor
        // return !m_simObjectDectector.get(); // Simulated object detector (commented out)
        return false; // Always false in simulation
    }

    /**
     * Stops the simulated gripper motor.
     */
    public void stopMotor() {
        if (m_simgripperMotor != null) {
            m_simgripperMotor.set(0); // Stop the simulated motor
        }
    }

    /**
     * Sets the simulated motor to intake speed.
     */
    public void setMotorIn() {
        if (m_simgripperMotor != null) {
            // Simulated motor in
            m_simgripperMotor.set(GripperConstants.motorSpeedInGripper); // Set simulated motor to intake speed
        }
    }

    /**
     * Sets the simulated motor to outtake speed.
     */
    public void setMotorOut() {
        if (m_simgripperMotor != null) {
            m_simgripperMotor.set(GripperConstants.motorSpeedOutGripper); // Set simulated motor to outtake speed
        }
    }

    /**
     * Creates a command to intake with the simulated gripper.
     *
     * @return The intake command.
     */
    @Override
    public Command Intake() {
        return new InstantCommand(() -> {
            setMotorIn(); // Set simulated motor to intake speed
        });
    }

    /**
     * Creates a command to outtake with the simulated gripper.
     *
     * @return The outtake command.
     */
    @Override
    public Command outtake() {
        return new InstantCommand(() -> {
            setMotorOut(); // Set simulated motor to outtake speed
        });
    }

    /**
     * Sets the motor speed directly. (Not implemented in simulation)
     * @param speed the speed to set the motor to.
     */
    @Override
    public void setMotorSpeed(double speed) {} // Not implemented in simulation

    /**
     * Gets the motor encoder position. (Always returns 0 in simulation)
     * @return the encoder position.
     */
    @Override
    public double getMotorPos() {return 0;} // Always returns 0 in simulation
}