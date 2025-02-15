package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private GripperIO io;
    private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();


    public Gripper(GripperIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Gripper Position", inputs.gripperPositionDegrees);
        Logger.recordOutput("Gripper Velocity", inputs.gripperVelocityRadPerSec);
        Logger.recordOutput("Gripper Current", inputs.gripperAppliedVolts);
        SmartDashboard.putNumber("Gripper Position", inputs.gripperPositionDegrees);
        SmartDashboard.putNumber("Gripper Velocity", inputs.gripperVelocityRadPerSec);
        SmartDashboard.putNumber("Gripper Current", inputs.gripperAppliedVolts);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double getVoltage(double voltage) {
        return io.getVoltage();
    }

    public Command stop() {
        return new FunctionalCommand(
                () -> {
                }, () -> io.setVoltage(0), (stop) -> io.stopMotor(), () -> false, this);
    }

    public Command Intake() {
        return new FunctionalCommand(
                () -> {
                },
                () -> io.setVoltage(GripperConstants.motorSpeedInGripper),
                (stop) -> setVoltage(0),
                () -> io.isCoralPresent(),
                this);

    }

    public Command outtake() {
        return new FunctionalCommand(
                () -> {
                },
                () -> io.setVoltage(GripperConstants.motorSpeedOutGripper),
                (stop) -> setVoltage(0),
                () -> io.isCoralPresent(),
                this);

    }

}
