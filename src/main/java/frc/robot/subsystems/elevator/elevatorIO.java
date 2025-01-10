package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;


public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        private final CANSparkMax motor;
        private final ProfiledPIDController pidController = new ProfiledPIDController(double kP, double kI, double kD, double MOVEMENT_CONSTRAINTS); 
        private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(double kS, double kG, double kV, double kA);
        
    }
    
    //delete turn motor??
    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator motor at the specified voltage. */
    public default void setElevatorVoltage(double volts) {}

    /** get elevator voltage. */
    public default double getElevatorVoltage() {
        return 0.0;
    }

    /** Enable or disable brake mode on the elevator motor. */
    public default void setElevatorBrakeMode(boolean enable) {}

    public default void setElevatorPIDFF(double p, double i, double d, double ff) {}

    public default void setElevatorVelocity(double velocityRadPerSec) {}

    public default double getAbsoluteEncoderOffset() {
        return 0.0;
    }

    public default void setElevatorCurrentLimit(int limit) {}
}
