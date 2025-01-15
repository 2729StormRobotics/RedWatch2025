package frc.robot.subsystems.hanger;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static frc.robot.subsystems.hanger.HangerConstants.MAX_VOLTAGE;

import org.littletonrobotics.junction.AutoLog;


public interface HangerIO {
    @AutoLog
    public static class HangerIOInputs{

    }

    public default void updateInputs(HangerIOInputs inputs) {
        
    }

    //runs hanger at specified voltage
    public default void setHangerVoltage(double volts) {

    }

    //gets current voltage of hanger
    public default double getHangerVoltage() {
        return 0.0;
    }

    public default void setHangerCurrentLimit(int limit) {}

}
