package frc.robot.subsystems.Gripper;

public interface GripperIO {
    
    public default boolean isCoralPresent() {return false;}

    public default void stop() {}

    public default void setIn() {}

    public default void setOut() {}
}
