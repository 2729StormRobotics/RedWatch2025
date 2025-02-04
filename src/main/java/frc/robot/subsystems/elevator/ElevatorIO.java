package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.subsystems.elevator.ElevatorConstants.kPElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kIElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kDElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kSElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kGElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kVElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
import static frc.robot.subsystems.elevator.ElevatorConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.subsystems.elevator.ElevatorConstants.kAElevator;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        // public TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        // public ProfiledPIDController elevatorPIDController = new ProfiledPIDController(kPElevator, kIElevator, kDElevator, MOVEMENT_CONSTRAINTS); //dont know have to fix later
        public ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kSElevator, kGElevator, kVElevator, kAElevator);
        
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps;
        public double elevatorPositionRad  = 0.0;
        public double kWheelDiameterMeters = 0.0;
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMeterPerSec = 0.0;
        public double elevatorVelocityRadPerSec = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator motor at the specified voltage. */
    public default void setElevatorVoltage(double volts) {}
    public default void setElevatorPower(double power) {}

    /** get elevator voltage. */
    public default double getElevatorVoltage() {
        return 0.0;
    }

    public default void setElevatorHeight(double targetHeight) {}
    
    /** Enable or disable brake mode on the elevator motor. */
    public default void setElevatorBrakeMode(boolean enable) {}

    public default void setElevatorPIDFF(double p, double i, double d, double ff) {}

    public default void setElevatorVelocity(double velocityRadPerSec) {}

    public default double getAbsoluteEncoderOffset() {
        return 0.0;
    }

    public default void stop() {}

    /**
     * Sets the proportional constant for velocity control.
     * 
     * @param p the proportional constant
     */
    public default void setP(double p) {}
    
    /**
     * Sets the integral constant for velocity control.
     * 
     * @param i the integral constant
     */
    public default void setI(double i) {}

    /**
     * Sets the derivative constant for velocity control.
     * 
     * @param d the derivative constant
     */
    public default void setD(double d) {}

    /**
     * Retrieves the proportional constant for velocity control.
     * 
     * @return the proportional constant
     */
    public default double getP() { return 0.0; }

    /**
     * Retrieves the integral constant for velocity control.
     * 
     * @return the integral constant
     */
    public default double getI() { return 0.0; }

    /**
     * Retrieves the derivative constant for velocity control.
     * 
     * @return the derivative constant
     */
    public default double getD() { return 0.0; }


}
