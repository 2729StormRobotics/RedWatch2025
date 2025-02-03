package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.drive.DashboardValues;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private LoggedNetworkNumber logP;
    private LoggedNetworkNumber logI;
    private LoggedNetworkNumber logD;

    // State of the note in the intake
    public enum ElevatorState {
        L1(ElevatorConstants.L1), // Not far enough in the intake or not in there at all
        L2(ElevatorConstants.L2), // just the right position in intake
        L3(ElevatorConstants.L3), // 
        L4(ElevatorConstants.L4),
        INTAKE(ElevatorConstants.INTAKE);
        public double elevatorState;

        private ElevatorState(Double elevatorState) {
            this.elevatorState = elevatorState;
        }
    }

    private ElevatorState noteState = ElevatorState.L1;
    private double currentVoltage = 0;
    
    public Elevator (ElevatorIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
        logP = new LoggedNetworkNumber("/SmartDashboard/Elevator/P", io.getP());
        logI = new LoggedNetworkNumber("/SmartDashboard/Elevator/I", io.getI());
        logD = new LoggedNetworkNumber("/SmartDashboard/Elevator/D", io.getD());
    }

    @AutoLogOutput(key = "Elevator/Close")
    public boolean isVoltageClose(double setVoltage) {
        double voltageDifference = Math.abs(setVoltage - inputs.elevatorAppliedVolts);
        return voltageDifference <= ElevatorConstants.ELEVATOR_TOLERANCE;
    }

    public void periodic() {
        io.updateInputs(inputs);
        // Update PID constants to ensure they are up to date
        // Logger.processInputs("/SmartDashboard/drive", inputs);

        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/State", noteState.name());
        Logger.recordOutput("Elevator/IndexerMotorConnected", inputs.elevatorVelocityRadPerSec != 0);
    }

    public void setVoltage(double voltage) {
        if (DashboardValues.turboMode.get()) {
            io.setElevatorVoltage(0);
        } else {
            io.setElevatorVoltage(voltage);
        }
        isVoltageClose(voltage);
    }

    public void setBrake(boolean brake) {
        io.setElevatorBrakeMode(brake);
    }

    // Sets motor speed based on where the note is in the intake
    public void runElevatorLoop() {
        setVoltage(currentVoltage);
    }
    /**
     * Uses input from controller to set speed of the flywheel
     * and is used as the default command for the ground intake
    */
    public Command speedCommand(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setElevatorVelocity(speed.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }
    // Allows manual command of the flywheel for testing
    public Command manualCommand(DoubleSupplier voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setElevatorVoltage(voltage.getAsDouble()),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

    public Command manualCommand(double voltage) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setElevatorVoltage(voltage),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }

    public Command stop() {
        return new FunctionalCommand(
            () -> {},
            () -> io.setElevatorVoltage(0),
            (stop) -> io.stop(),
            () -> false,
            this
        );
      }
    public Command goToPosition(ElevatorState state) {
        return new FunctionalCommand(
            () -> {},
            () -> io.setElevatorHeight(state.elevatorState),
            (stop) -> io.stop(),
            () -> false,
            this
        );
    }
}
