package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.LED.BlinkinLEDController.BlinkinPattern;

import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
    private GripperIO io;
    private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();
    private BlinkinLEDController ledController;

    public Gripper(GripperIO io) {

        this.io = io;
        ledController = BlinkinLEDController.getInstance();

        SmartDashboard.putData(getName(), this);
    }

    @Override
    public void periodic() {
        if (io.isCoralPresent()) {
            ledController.setPattern(BlinkinPattern.RAINBOW_FOREST_PALETTE);
        }
        io.updateInputs(inputs);
        Logger.recordOutput("Gripper Position", inputs.gripperPositionDegrees);
        Logger.recordOutput("Gripper Velocity", inputs.gripperVelocityRadPerSec);
        Logger.recordOutput("Gripper Current", inputs.gripperAppliedVolts);
        SmartDashboard.putNumber("Gripper/Position", inputs.gripperPositionDegrees);
        SmartDashboard.putNumber("Gripper/Velocity", inputs.gripperVelocityRadPerSec);
        SmartDashboard.putNumber("Gripper/Current", inputs.gripperAppliedVolts);
        SmartDashboard.putBoolean("Gripper/IsCoralDetected", io.isCoralPresent());
    }

    public double getVoltage(double voltage) {
        return io.getVoltage();
    }

    public Command stop() {
        return new FunctionalCommand(
                () -> {
                },
                () -> io.stop(),
                (stop) -> io.stop(),
                () -> false,
                this);
    }

    public Command AutoIntake() {
        return new FunctionalCommand(
                () -> {
                    ledController.setPattern(BlinkinPattern.GREEN);
                },
                () -> io.setMotorIn(),
                (stop) -> {
                    io.stop();
                    ledController.setPattern(BlinkinPattern.FIRE_MEDIUM);
                },
                () -> false,
                this);

    }

    public Command AutoOuttake() {
        return new FunctionalCommand(
                () -> {
                    ledController.setPattern(BlinkinPattern.RED);
                },
                () -> io.setMotorOut(),
                (stop) -> {
                    io.stop();
                    ledController.setPattern(BlinkinPattern.FIRE_MEDIUM);
                },
                () -> false,
                this);
    }

    public Command Intake() {
        return new FunctionalCommand(
                () -> {
                    ledController.setPattern(BlinkinPattern.GREEN);
                },
                () -> io.setMotorIn(),
                (stop) -> {
                    io.stop();
                    ledController.setPattern(BlinkinPattern.FIRE_MEDIUM);
                },
                () -> io.isCoralPresent(),
                this);

    }

    public Command outtake() {
        return new FunctionalCommand(
                () -> {
                    ledController.setPattern(BlinkinPattern.RED);
                },
                () -> io.setMotorOut(),
                (stop) -> {
                    io.stop();
                    ledController.setPattern(BlinkinPattern.FIRE_MEDIUM);
                },
                () -> !io.isCoralPresent(),
                this);

    }

    public Command reverse() {
        return new FunctionalCommand(
                () -> {
                    ledController.setPattern(BlinkinPattern.RED);
                },
                () -> io.reverse(),
                (stop) -> {
                    io.stop();
                    ledController.setPattern(BlinkinPattern.FIRE_MEDIUM);
                },
                () -> false,
                this);

    }

}
