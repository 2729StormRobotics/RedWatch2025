package frc.robot.subsystems.hanger;

import org.littletonrobotics.junction.AutoLog;

public interface HangerIO {

    public default void stop() {
    }

    public default void pull() {
    }

    public default void release() {
    }

    public default void setHangerVoltage(double volts) {
    }

    public default double getHangerVoltage() {
        return 0;
    }

    public default double getHangerAngle() {
        return 0;
    }

    public default void setHangerCurrentLimit(int limit) {
    }
}
