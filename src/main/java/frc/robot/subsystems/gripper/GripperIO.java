package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface GripperIO {

    public default boolean isCoralPresent() {
        return false;
    }

    public default void setVoltage( double voltage ){}

    public default double getVoltage(){return 0.0;}

    public default void stopMotor() {
    }

    public default void setIn() {
    }

    public default void setOut() {
    }

    public default Command intake() {
        return null;
    }

    public default Command outtake() {
        return null;
    }

    public default Command stop(){
        return null;
 
    }

    @AutoLog
    public static class GripperIOInputs {
        // public ClosedLoopConfig armCLC = new ClosedLoopConfig();
        public double gripperAppliedVolts = 0.0;
        public double gripperPositionRad = 0.0;
        public double gripperPositionDegrees = 0.0;
        public double gripperVelocityRadPerSec = 0.0;

    }

      public default void updateInputs(GripperIOInputs inputs) {}
}