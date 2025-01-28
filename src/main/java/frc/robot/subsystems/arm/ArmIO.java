package frc.robot.subsystems.arm;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.arm.ArmIOConstants;


public interface ArmIO {

    public default void pivotClockwise() {};
    public default void pivotCounterclockwise() {};
    public default void setArmPosition() {};
    public default void stopArm() {};

    public static class ArmIOInputs {
        public TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(ArmIOConstants.MAX_VELOCITY_METERS_PER_SECOND, ArmIOConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public ProfiledPIDController armPIDController = new ProfiledPIDController(ArmIOConstants.kPArm, ArmIOConstants.kIArm, ArmIOConstants.kDArm, MOVEMENT_CONSTRAINTS); //dont know have to fix later

        public double armAppliedVolts = 0.0;
        public double[] armCurrentAmps;
        public double armPositionRad  = 0.0;
        public double armPositionAngle = 0.0;
        public double armVelocityMeterPerSec = 0.0;
        public double armVelocityRadPerSec = 0.0;

    }
    public default void updateInputs(ArmIOInputs inputs) {}

}
