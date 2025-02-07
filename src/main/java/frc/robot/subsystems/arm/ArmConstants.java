package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int kArmCANID = 0;
    public static final int kArmCANID2 = 1;

    public static final double kArmMotorSpeed = 0.2;



  public static final double PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double PID_VELOCITY_TOLERANCE = 0.5;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO

    public static final double kArmMotorStop = 0;

    public static final double kL1 = 0.0;
    public static final double kL2 = 0.0;
    public static final double kL3 = 0.0;
    public static final double kL4 = 0.0;
    public static final double kIntake = 0.0;
    public static final double kOneMore = 0; // ?????

    public static double kPArm = 0.0;
    public static double kIArm = 0.0;
    public static double kDArm = 0.0;
    public static double kArmMinOutputPower = 0.0;
    public static double kArmMaxOutputPower = 1.0;

    public static class ArmSimConstants {
        public static final double[] kArmSimPID = { 15, 0, 0, 0 };

        public static final int kMotorPort = 2;
        public static final int kEncoderAChannel = 2;
        public static final int kEncoderBChannel = 3;

        // The P gain for the PID controller that drives this arm.
        public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

        // distance per pulse = (angle per revolution) / (pulses per revolution)
        // = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 1 / 4096;

        public static final double kArmReduction = 200;
        public static final double kArmMass = 10.0; // Kilograms
        public static final double kArmLength = Units.inchesToMeters(20);
        public static final double kMinAngleRads = Units.degreesToRadians(0);
        public static final double kMaxAngleRads = Units.degreesToRadians(180);
    }

}