package frc.robot.subsystems.hanger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HangerIOSim implements HangerIO{
    private static final double LOOP_PERIOD_SECS = 0.02;
    private static final DCMotor neoMotor = DCMotor.getNeo550(1); //Change to whatever CAN ID it is 

    private final DCMotorSim turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(neoMotor, 0.004, turnReduction),neoMotor);
    private double appliedVolts = 0.0;

 public void updateInputs(HangerIOInputs inputs) {

    turnSim.update(LOOP_PERIOD_SECS);
    
 }

                                    
@Override
public void startMotor(){
    neoMotor.set(0.2);//Change to whatever speed wanted 
    
 }

@Override
public void reverseMotor(){
    neoMotor.set(-0.2)//Change to whatever speed wanted 

}

@Override
public void stopMotor (){
    neoMotor.set(0);

 }

@Override                                    
  public void setMotorVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(appliedVolts);
  }

}
