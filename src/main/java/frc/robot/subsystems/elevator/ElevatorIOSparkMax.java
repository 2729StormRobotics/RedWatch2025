package frc.robot.subsystems.elevator;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOSparkMax {
    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysidPositionMeasure = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysidVelocityMeasure = MutableMeasure.mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

}
