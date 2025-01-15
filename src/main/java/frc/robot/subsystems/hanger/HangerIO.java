package frc.robot.subsystems.hanger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.subsystems.hanger.HangerConstants.MAX_VOLTAGE;

import org.littletonrobotics.junction.AutoLog;


public class HangerIO extends SubsystemBase {

    public static SparkFlex m_hangerMotor = new SparkFlex(HangerConstants.hangerMotorPort, MotorType.kBrushless); 
    public static boolean isClosed = false;
    public static Timer timer = new Timer();

        public static class HangerIOInputs{
    
        }
    
        public void stop() {
            m_hangerMotor.set(0);
        }
        
        public static void pull() {
            m_hangerMotor.set(HangerConstants.motorSpeedOpenHanger);
            timer.delay(0.5);
            m_hangerMotor.set(0);
            isClosed = true;
        }


        public static void release()
        {
            m_hangerMotor.set(-HangerConstants.motorSpeedOpenHanger);
            timer.delay(0.5);
            m_hangerMotor.set(0);
            isClosed = false;
        }

    //public void updateInputs(HangerIOInputs inputs) {
        
    //}

    //runs hanger at specified voltage
    public void setHangerVoltage(double volts) {

    }

    //gets current voltage of hanger
    public double getHangerVoltage() {
        return 0.0;
    }

    public void setHangerCurrentLimit(int limit) {}

}
