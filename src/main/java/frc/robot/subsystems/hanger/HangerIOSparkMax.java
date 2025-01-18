package frc.robot.subsystems.hanger;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;



public class HangerIOSparkMax implements HangerIO{


    public static SparkMax m_hangerMotor = new SparkMax(HangerConstants.hangerMotorPort, MotorType.kBrushless); 
    public static boolean isClosed = false;
    public static Timer timer = new Timer();
    
    @Override
    public void stop() {
        m_hangerMotor.stopMotor();
        
    }

    @Override
    public void pull() {
        m_hangerMotor.set(HangerConstants.motorSpeedOpenHanger);
        timer.delay(0.5);
        //not sure if this will lock the motor
        m_hangerMotor.set(0);
        isClosed = true;
    }

    @Override
    public void release() {
        m_hangerMotor.set(-HangerConstants.motorSpeedOpenHanger);
        timer.delay(0.5);
        m_hangerMotor.set(0);
        isClosed = false;

    }

    @Override
    public boolean getClosed(){
        return isClosed;
    }
    
}