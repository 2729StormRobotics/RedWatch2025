package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public interface ArmIO {

    public default void pivotClockwise() {
    };

    public default void pivotCounterclockwise() {
    };

    public default void setArmPosition(double position) {
    };

    public default void stopArm() {
    };

    public default void changeOffset(double newOffset){

    }

    public default boolean getHallEffect(){return false;};

    public default double getVoltage() {
        return 0.0;
    };

    public default double getArmAngleRad() {
        return 0.0;
    };

    public default double getArmAngleDegrees() {
        return 0.0;
    };

    public default double getArmVelocity() {
        return 0.0;
    }; // i forgot the units

    public default void setVoltage(double voltage) {
    };
    
    public default void setSpeed(double speed) {
    };

    public default void setP(double p) {
    };

    public default void setI(double i) {
    };

    public default void setD(double d) {
    };

    public default double getP() {
        return 0.0;
    };

    public default double getI() {
        return 0.0;
    };

    public default double getD() {
        return 0.0;
    };

    @AutoLog
    public static class ArmIOInputs {
        // public ClosedLoopConfig armCLC = new ClosedLoopConfig();
        public double armAppliedVolts = 0.0;
        public double armPositionRad = 0.0;
        public double armPositionDegrees = 0.0;
        public double armVelocityRadPerSec = 0.0;

    }

      public default void updateInputs(ArmIOInputs inputs) {}


}