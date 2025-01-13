package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DigitalInput; // For Beambreak
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class GripperIO extends SubsystemBase {
    private DigitalInput m_ObjectDectector = new DigitalInput(GripperConstants.beamBreakPort);
    private SparkMax m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless); 
    private boolean isOpen = false;
    private Timer timer; // For 2 sec of motor run


    
    public GripperIO () {
        
    }

    public void stop() {
        m_gripperMotor.set(0);
    }

    public void open() {
        m_gripperMotor.set(GripperConstants.motorSpeedOpenGripper);
        timer.reset();
        timer.start();
        while (timer.get() < 2.0) {}
        m_gripperMotor.set(0);
        isOpen = true;
    }

    public void close() {
        m_gripperMotor.set(GripperConstants.motorSpeedCloseGripper);
        timer.reset();
        timer.start();
        while (timer.get() < 2.0) {}
        m_gripperMotor.set(0);
        isOpen = false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Gripper: ", isOpen);
        SmartDashboard.putBoolean("Beambreak: ", m_ObjectDectector.get());
        SmartDashboard.putNumber("Speed: ", m_gripperMotor.get());
    }
}
