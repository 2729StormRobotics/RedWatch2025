package frc.robot.subsystems.gripper;

/**
 * Constants for the Gripper subsystem.
 */
public class GripperConstants {
    public static final int gripperMotorPort = 13; // Port for the gripper motor

    public static final int beambreakPort = 2; // Port for the beambreak sensor (to be updated after testing)

    public static final double motorSpeedInGripper = -0.15; // Motor speed for intaking (to be updated after testing)

    public static final double motorSpeedOutGripper = -1; // Motor speed for outtaking

    public static final double kPGripper = 0.003; // Proportional gain for gripper position control
    public static final double kIGripper = 0; // Integral gain for gripper position control
    public static final double kDGripper = 0; // Derivative gain for gripper position control
}