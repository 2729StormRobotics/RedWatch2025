package frc.robot.subsystems.arm;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmIOConstants;



public interface ArmIO {

    public default void pivotClockwise() {};
    public default void pivotCounterclockwise() {};
    public default void setArmPosition() {};
    public default void stopArm() {};
    public default double getVoltage(){return 0.0;};
    public default double getArmAngleRad(){return 0.0;};
    public default double getArmAngleDegrees() {return 0.0;};
    public default double getArmVelocity() {return 0.0;}; // i forgot the units


    public default SequentialCommandGroup clockwise(){return new SequentialCommandGroup(null);};
    public default SequentialCommandGroup counterClockwise(){return new SequentialCommandGroup(null);};
    public default SequentialCommandGroup stop(){return new SequentialCommandGroup(null);};
    public default SequentialCommandGroup armPosition(){return new SequentialCommandGroup(null);};


    public static class ArmIOInputs {
        //public TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(ArmIOConstants.MAX_VELOCITY_METERS_PER_SECOND, ArmIOConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        //public ProfiledPIDController armPIDController = new ProfiledPIDController(ArmIOConstants.kPArm, ArmIOConstants.kIArm, ArmIOConstants.kDArm, MOVEMENT_CONSTRAINTS); //dont know have to fix later
        public ClosedLoopConfig armCLC = new ClosedLoopConfig();
        public static double armAppliedVolts = 0.0;
        public static double armPositionRad  = 0.0;
        public static double armPositionDegrees = 0.0;
        public static double armVelocityMeterPerSec = 0.0;
        public static double armVelocityRadPerSec = 0.0;

    }
    public default void updateInputs(ArmIOInputs inputs) {}

}
