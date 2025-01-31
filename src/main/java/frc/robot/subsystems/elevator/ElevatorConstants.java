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
    public static final double kElevatorMinOutputPower = 0.0;
    public static final double kElevatorMaxOutputPower = 0.0;
    
    public static final double ELEVATOR_TOLERANCE = 1;
    //add MOVEMENT CONSTRAINTS
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
    public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    // SPARK MAX CAN IDs
    // Subject to change depending on what robot side is the front
    public static final int kLeftElevatorCanId = 9; //ask controls
    public static final int kRightElevatorCanId = 10; //ask controls

    //string pot info
    public static final int kStringPotPort = 0;

    //height constants'
    public static final double MIN_HEIGHT = 0.0;
    public static final double MAX_HEIGHT = 10.0;
    public static final double L1 = 1.0;
    public static final double L2 = 2.0;
    public static final double L3 = 3.0;
    public static final double L4 = 4.0;
    public static final double INTAKE = 4.0;

    public static final double offsetAdd = 0;
    public static final double offsetMult = 0;
    
    //motor speeds
    public static final double kMotorSpeedUp = 0.0;
    public static final double kMotorSpeedDown = -kMotorSpeedUp;

    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps;
    public double elevatorPositionRad  = 0.0;
    public double kWheelDiameterMeters = 0.03;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMeterPerSec = 0.0;
    public double elevatorVelocityRadPerSec = 0.0;
}
