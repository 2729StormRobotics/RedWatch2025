package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;

public interface GripperIO {

    public default boolean isCoralPresent() {
        return false;
    }

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
}