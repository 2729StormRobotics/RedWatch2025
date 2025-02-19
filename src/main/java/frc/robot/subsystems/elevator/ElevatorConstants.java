package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static double STRINGPOT_POSITION_CONVERSION_FACTOR = 6.6292;

    public static double stringPottoElevatorConversion = 1;
    public static double ElevatorOffset = 1;

    // PID values for elevator to be tested
    public static final double kPElevator = 0.001;
    public static final double kIElevator = 0.0;
    public static final double kDElevator = 0.0;
    public static final double kSElevator = 0.0;
    public static final double kGElevator = 0.0;
    public static final double kVElevator = 0.0;
    public static final double kAElevator = 0.0;

    public static final double ELEVATOR_TOLERANCE = 1;
    // add MOVEMENT CONSTRAINTS
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // SPARK MAX CAN IDs
    // Subject to change depending on what robot side is the front
    public static final int kLeftElevatorCanId = 9; // ask controls
    public static final int kRightElevatorCanId = 10; // ask controls

    // string pot info
    public static final int kStringPotPort = 0;

    // height constants'
    public static final double MIN_HEIGHT = 0.0;
    public static final double MAX_HEIGHT = 10.0;
    public static final double L1 = 2.4;
    public static final double L2 = 5.1;
    public static final double L3 = 14.3;
    public static final double L4 = 28.85;
    public static final double INTAKE = 4.0;



    public static final double ELEVATOR_PID_TOLERANCE = Units.degreesToRadians(1);
    public static final double ELEVATOR_PID_VELOCITY_TOLERANCE = 0.5;

    // top height 50 inches (measured from bottom of elevator frame)
    // bottom height 12.375 inches (measured from bottom of frame)
    public static final double ELEVATOR_MAX_HEIGHT = 53; // not including chasis
    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double ELEVATOR_STARTING_HEIGHT = 0.1;

    // sysid stuff
    public static final double RAMP_RATE = 0.5;
    public static final double STEP_VOLTAGE = 3.0;

    public static int LEFT_MOTOR_ID = 9;
    public static int RIGHT_MOTOR_ID = 10;
    public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
    /** Used for converting angular displacement into linear displacement */
    public static double MOTOR_RADIUS_METERS = 1.0;
    /** Gear ratio of the elevator motors */
    public static double GEAR_RATIO = 1.0;

    /** Final position conversion factor based on drum radius and gear ratio */
    public static double POSITION_CONVERSION_FACTOR = 2 * 3.14 * ElevatorConstants.MOTOR_RADIUS_METERS * GEAR_RATIO;

    /** When the elevator is on the bottom, what does the encoder say */
    public static double ELEVATOR_OFFSET_METERS = 0.0;
    /** Tolerance used when checking if the elevator is at the setpoint */
    public static double SETPOINT_TOLERANCE_METERS = 0.2;

    public static final double[] kElevatorRealPID = { 1.45, 0, 0, 0 };

    public static final double ELEVATOR_MASS_KG = 5;

    public static class ElevatorSimConstants {
        public static final double[] kElevatorSimPID = { 15, 0, 0, 0 };
        public static final int kEncoderAChannel = 7;
        public static final int kEncoderBChannel = 8;
        // Convert from encoder steps to meters

        // 4096 pulses per revolution
        // (2pi radians / 4096) * gear ratio
        public static final double ENCODER_DIST_PER_PULSE = 2 * Math.PI / 4096 * MOTOR_RADIUS_METERS * GEAR_RATIO;
        // public static final int kMotorPort = 2;

    }
}
