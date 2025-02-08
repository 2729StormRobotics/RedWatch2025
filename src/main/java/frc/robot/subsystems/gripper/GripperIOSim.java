package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.DigitalInput; // For Beambreak
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class GripperIOSim implements GripperIO {
    private DigitalInput m_simObjectDectector;
    private SparkMax m_simgripperMotor;

    public void GripperIO(boolean isSimulation) {
        if (isSimulation) {
            m_simObjectDectector = new DigitalInput(GripperConstants.beambreakPort);
            m_simgripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);
        }

    }

    public boolean isCoralPresent() {
        // Have to change for the promxity sensor
        return !m_simObjectDectector.get();
    }

    public void stopMotor() {
        if (m_simgripperMotor != null) {
            m_simgripperMotor.set(0);

        }
    }

    public void setMotorIn() {
        if (m_simgripperMotor != null) {
            // Simulated motor in
            m_simgripperMotor.set(GripperConstants.motorSpeedInGripper);
        }

    }

    public void setMotorOut() {
        if (m_simgripperMotor != null) {
            m_simgripperMotor.set(GripperConstants.motorSpeedOutGripper);
        }
    }

    @Override
    public Command intake() {
        return new InstantCommand(() -> {
            setMotorIn();
        });
    }

    @Override
    public Command outtake() {
        return new InstantCommand(() -> {
            setMotorOut();
        });
    }
}