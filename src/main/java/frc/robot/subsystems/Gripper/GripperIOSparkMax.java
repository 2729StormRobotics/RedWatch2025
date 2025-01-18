package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class GripperIOSparkMax implements GripperIO {
    private SparkMax m_gripperMotor;
    private DigitalInput m_objectDetector;

    public GripperIOSparkMax() {
        m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);
        m_objectDetector = new DigitalInput(GripperConstants.proxmitySensorPort);
        stop();
    }

    @Override
    public boolean isCoralPresent() {
        return !m_objectDetector.get();
    }

    @Override
    public void stop() {
        m_gripperMotor.set(0);
    }

    @Override
    public void setIn() {
        m_gripperMotor.set(GripperConstants.motorSpeedInGripper);
    }
    
    @Override
    public void setOut() { 
        m_gripperMotor.set(GripperConstants.motorSpeedOutGripper);
    }
}
