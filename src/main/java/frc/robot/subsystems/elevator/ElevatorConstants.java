package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ElevatorConstants {
    // PID values for elevator to be tested
    public static final double kPElevator = 0.0;
    public static final double kIElevator = 0.0;
    public static final double kDElevator = 0.0;
    public static final double kSElevator = 0.0;
    public static final double kGElevator = 0.0;
    public static final double kVElevator = 0.0;
    public static final double kAElevator = 0.0;
    
    //add MOVEMENT CONSTRAINTS
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // SPARK MAX CAN IDs
    // Subject to change depending on what robot side is the front
    public static final int kLeftElevatorCanId = -1; //ask controls
    public static final int kRightElevatorCanId = -2; //ask controls

    //height constants'
    public static final double MIN_HEIGHT = 0.0;
    public static final double MAX_HEIGHT = 0.0;
    public static final double L1 = 0.0;
    public static final double L2 = 0.0;
    public static final double L3 = 0.0;
    public static final double L4 = 0.0;

    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps;
    public double elevatorPositionRad  = 0.0;
    public double kWheelDiameterMeters = 0.0;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMeterPerSec = 0.0;
    public double elevatorVelocityRadPerSec = 0.0;
}
