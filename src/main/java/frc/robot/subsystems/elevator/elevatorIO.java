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

    /** get elevator voltage. */
    public default double getElevatorVoltage() {
        return 0.0;
    }

    public default void setElevatorHeight(double targetHeight, ElevatorIOInputs inputs) {
        // Ensure target is within the allowed range
        if (targetHeight < 0 || targetHeight > inputs.kWheelDiameterMeters) {
            System.out.println("Target height is out of range: " + targetHeight);
            return;
        }
    
        // Update the PID controller's goal
        // inputs.elevatorPIDController.setGoal(targetHeight);
    
        // // Calculate the PID output and feedforward voltage
        // double pidOutput = inputs.elevatorPIDController.calculate(inputs.elevatorPositionMeters);
        double feedforward = inputs.feedforwardController.calculate(inputs.elevatorVelocityMeterPerSec);
    
        // Compute total voltage to apply
        // double appliedVoltage = pidOutput + feedforward;
    
        // Apply the computed voltage
        // setElevatorVoltage(appliedVoltage);
    }
    

    /** Enable or disable brake mode on the elevator motor. */
    public default void setElevatorBrakeMode(boolean enable) {}

    public default void setElevatorPIDFF(double p, double i, double d, double ff) {}

    public default void setElevatorVelocity(double velocityRadPerSec) {}

    public default double getAbsoluteEncoderOffset() {
        return 0.0;
    }


}
