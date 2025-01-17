package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DigitalInput; // For Beambreak
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Gripper.GripperIO;

public class GripperIOSim implements GripperIO {
    private DigitalInput m_simObjectDectector;
    private SparkMax m_simgripperMotor;


    public void GripperIO(boolean isSimulation) {
        if (isSimulation) {
            m_simObjectDectector =  new DigitalInput(GripperConstants.proxmitySensorPort);
            m_simgripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);
        }

    }

    public boolean isCoralPresent() {
        if (m_simObjectDectector != null) {
            //Have to change for the promxity sensor
            return !m_simObjectDectector.get();
        }
    }


    public void stop() {
        if (m_simgripperMotor != null) {
         m_simgripperMotor.set(0);
    
        }
    }

    public void setIn() {
        if (m_simgripperMotor != null) {
            // Simulated motor in
            m_simgripperMotor.set(GripperConstants.motorSpeedInGripper);
        }
    
        }

    public void setOut() { 
        if (m_simgripperMotor != null) {
            m_simgripperMotor.set(GripperConstants.motorSpeedOutGripper);
            while (isCoralPresent()) {} // Logic should work     yes (hopefully)
            stop();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Proximity: ", m_simObjectDectector.get());
        SmartDashboard.putNumber("Speed: ", m_simgripperMotor.get());
    }
}
