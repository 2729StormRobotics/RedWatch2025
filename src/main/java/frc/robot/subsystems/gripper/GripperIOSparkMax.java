package frc.robot.subsystems.gripper;

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

    private DigitalInput m_input;

    public GripperIOSparkMax() {
        m_input = new DigitalInput(GripperConstants.beambreakPort);
        m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);
        m_objectDetector = new DigitalInput(GripperConstants.beambreakPort);

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
    public void stopMotor() {
        m_gripperMotor.set(0);
    }

    @Override
    public void setMotorIn() {
        m_gripperMotor.set(GripperConstants.motorSpeedInGripper);
    }

    @Override
    public void setMotorOut() {
        m_gripperMotor.set(GripperConstants.motorSpeedOutGripper);
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

    @Override
    public Command stop(){
        return new InstantCommand(() -> {
            stopMotor();
        });
    }
}