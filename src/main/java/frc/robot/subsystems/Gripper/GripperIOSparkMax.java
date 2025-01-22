package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class GripperIOSparkMax implements GripperIO {
    private SparkMax m_gripperMotor;
    private SparkMaxConfig motorConfig;

    private DigitalInput m_objectDetector;

    public GripperIOSparkMax() {
        m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);
        m_objectDetector = new DigitalInput(GripperConstants.proxmitySensorPort);

        // Configure Motor
        motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.pid(GripperConstants.kPGripper, GripperConstants.kIGripper, GripperConstants.kDGripper);
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.smartCurrentLimit(40);
        
        stop();
    }

    @Override
    public boolean isCoralPresent() {
        return m_objectDetector.get();
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

    @Override
    public Command intake() {
        return new InstantCommand(() -> {setIn();});
    }

    @Override
    public Command outtake() {
        return new InstantCommand(() -> {setOut();});
    }
}
