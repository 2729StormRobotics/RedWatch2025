package frc.robot.subsystems.hanger;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HangerIOSim implements HangerIO{

  private static final DCMotor neoMotor = DCMotor.getNeo550(1);

    private final DCMotorSim turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, turnReduction),
            turnMotorModel);

 public void updateInputs(HangerIOInputs inputs) {

    turnSim.update(LOOP_PERIOD_SECS);

@Override

public void robotInit(){

    motor = new DCMotor( );// replace with specfic motor instance if needed

    motorSim = new DCMotorSim(motor);
    
}

@Override

public void start(){


 }

@Override

public void stop (){


 }
}
