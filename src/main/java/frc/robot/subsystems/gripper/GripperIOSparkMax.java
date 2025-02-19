package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class GripperIOSparkMax implements GripperIO {
    private SparkMax m_gripperMotor;
    private SparkMaxConfig motorConfig;

    private SparkLimitSwitch m_objectDetector;

    public GripperIOSparkMax() {
        m_gripperMotor = new SparkMax(GripperConstants.gripperMotorPort, MotorType.kBrushless);
        m_objectDetector = m_gripperMotor.getForwardLimitSwitch();

        // Configure Motor
        motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.pid(GripperConstants.kPGripper, GripperConstants.kIGripper, GripperConstants.kDGripper);
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.smartCurrentLimit(40);

        m_gripperMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        stop();
    }

    /**
     * Returns if an object is present in gripper
     * @return boolean
     */
    @Override
    public boolean isCoralPresent() {
        return m_gripperMotor.getForwardLimitSwitch().isPressed();
    }

    /**
     * Sets motor speed to 0
     */
    @Override
    public void stopMotor() {
        m_gripperMotor.set(0);
    }

    /**
     * Sets motor speed to inwards
     */
    @Override
    public void setMotorIn() {
        m_gripperMotor.set(GripperConstants.motorSpeedInGripper);
    }

    /**
     * Sets motor speed to outwards
     */
    @Override
    public void setMotorOut() {
        m_gripperMotor.set(GripperConstants.motorSpeedOutGripper);
    }

    
}