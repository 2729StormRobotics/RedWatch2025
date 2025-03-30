package frc.robot.subsystems.hanger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hanger.HangerIOSparkMax.StateMachine;
import frc.robot.subsystems.hanger.HangerIOSparkMax.TransitionEvents;

/**
 * Interface for Hanger I/O implementations.
 */
public interface HangerIO {

  public default StateMachine getState(){
    return StateMachine.Docked;
  }

  public void setEvent(TransitionEvents event);

  public void setMotorRetract();

  public void setMotorExtend();

  public void stop();
}