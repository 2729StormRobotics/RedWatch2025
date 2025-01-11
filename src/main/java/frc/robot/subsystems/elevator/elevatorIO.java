package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static frc.robot.subsystems.elevator.ElevatorConstants.kPElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kIElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kDElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kSElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kGElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kVElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kAElevator;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        private final SparkMax motor;
        private final ProfiledPIDController pidController = new ProfiledPIDController(kPElevator, kIElevator, kDElevator, MOVEMENT_CONSTRAINTS); //dont know have to fix later
        private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kSElevator, kGElevator, kVElevator, kAElevator);
        public final AnalogPotentiometer elevatorPot = new AnalogPotentiometer(0); //idk change it later
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps;
        public double elevatorPositionRad  = 0.0;
        public double kWheelDiameterMeters = 0.0;
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMeterPerSec = 0.0;
        public double elevatorVelocityRadPerSec = 0.0;
        public final AnalogPotentiometer pot;
        public double pot_val;
    }

    public default void elevatorPot() {
        pot = new AnalogPotentiometer(1);
    }

    // Gets the distance from this value to this value.
    public default double getDistance(){
        return pot_val;
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
