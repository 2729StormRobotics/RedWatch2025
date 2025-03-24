package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    // CAN IDs for the arm motors.
    public static final int kArmCANID = 11;
    public static final int kArmCANID2 = 12;

    //  Maximum speed of the arm motor.
    public static final double kArmMotorSpeed = 0.2;

    //  Ramp rate and step voltage for system identification.
    public static final double RAMP_RATE = 0.5;
    public static final double STEP_VOLTAGE = 1.0;
    // Minimum and maximum allowed arm angles.
    public static final double ARM_MIN_ANGLE = 0.0;
    public static final double ARM_MAX_ANGLE = 180.0;

    //  PID control tolerances for arm position and velocity.
    public static final double PID_TOLERANCE = (1);
    public static final double PID_VELOCITY_TOLERANCE = 1;

    //  Maximum velocity and acceleration of the arm.
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO: Tune this value

    //  Value to stop the arm motor.
    public static final double kArmMotorStop = 0;

    //  Predefined arm positions.
    public static final double kSTOW = 90; //  Stow position.
    public static final double kL1 = 139.04;
    public static final double kL2 = 58;
    public static final double kL3 = 56.05;
    public static final double kL4 = 57.63;
    public static final double kAlgae = 43.4;
    public static final double kIntake = 110;
    public static final double kOneMore = 0; //  Purpose unclear, needs clarification.

    //  PID gains for arm control.
    public static double kPArm = 0.01;
    public static double kIArm = 0.0;
    public static double kDArm = 0.0;
     //  Minimum and maximum output power for the arm motor.
    public static double kArmMinOutputPower = -1.0;
    public static final double kArmMaxOutputPower = 1.0;

    // Constants used for arm simulation.
    public static class ArmSimConstants {
        public static final double[] kArmSimPID = { 15, 0, 0, 0 }; //  PID gains for simulation.

        public static final int kMotorPort = 2;
        public static final int kEncoderAChannel = 2;
        public static final int kEncoderBChannel = 3;

        //  Default arm setpoint in degrees.
        public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

        //  Encoder distance per pulse calculation.
        public static final double kArmEncoderDistPerPulse = 1 / 4096;

        //  Arm reduction, mass, and length for simulation.
        public static final double kArmReduction = 200;
        public static final double kArmMass = 10.0; // Kilograms
        public static final double kArmLength = Units.inchesToMeters(20);
         //  Minimum and maximum arm angles in radians for simulation.
        public static final double kMinAngleRads = Units.degreesToRadians(0);
        public static final double kMaxAngleRads = Units.degreesToRadians(180);
    }

}
