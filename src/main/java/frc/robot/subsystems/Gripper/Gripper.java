package frc.robot.subsystems.Gripper;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
    private final GripperIO io;

    private double currentVoltage = 0;

    public Gripper(GripperIO io) {
        this.io = io;
    }

    // @AutoLogOutput(key = "Gripper/Close")
    // public boolean isVoltageClose(double setVoltage) {
    // double voltageDifference = Math.abs(setVoltage - io.applied);
    // return voltageDifference <= GripperConstants.ELEVATOR_TOLERANCE;
    // }

    public void periodic() {
        // Update PID constants to ensure they are up to date
        // Logger.processInputs("/SmartDashboard/drive", inputs);
        SmartDashboard.putBoolean("CoralPresent?", isCoralPresent());
    }

    public boolean isCoralPresent() {
        return io.isCoralPresent();
    }

    public void stop() {
        io.stop();
    }

    public void setIn() {
        io.setIn();
    }

    public void setOut() {
        io.setOut();
    }

    public Command Intake() {
        return new FunctionalCommand(
                () -> {
                },
                () -> io.setIn(),
                (stop) -> io.stop(),
                () -> io.isCoralPresent(),
                this);
    }

    public Command Outtake() {
        return new FunctionalCommand(
                () -> {
                },
                () -> io.setOut(),
                (stop) -> io.stop(),
                () -> !io.isCoralPresent(),
                this);
    }
}
