package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DigitalInput; // For Beambreak
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class GripperIO extends SubsystemBase {
    private DigitalInput m_ObjectDectector = new DigitalInput(GripperConstants.proxmitySensorPort);
    private SparkMax m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);


    public boolean isCoralPresent() {
        return !m_ObjectDectector.get();
    }


    public void stop() {
        m_gripperMotor.set(0);
    }

    public void setIn() {
        m_gripperMotor.set(GripperConstants.motorSpeedInGripper);
        while (!isCoralPresent()) {} // Logic should work
        stop();
    }

    public void setOut() { 
        m_gripperMotor.set(GripperConstants.motorSpeedOutGripper);
        while (isCoralPresent()) {} // Logic should work     yes (hopefully)
        stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Proximity: ", m_ObjectDectector.get());
        SmartDashboard.putNumber("Speed: ", m_gripperMotor.get());
    }
}
