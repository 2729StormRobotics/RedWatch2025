package frc.robot.commands.ReefCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.gripper.*;


public class DispenseL1 extends ParallelCommandGroup {
    private Arm m_arm;
    private Elevator m_elevator;
    private Gripper m_gripper; 
  /** Creates a new DispenseL1. */
  public DispenseL1(Arm arm, Elevator elevator, Gripper gripper) {
    m_arm =arm;
    m_elevator = elevator;
    m_gripper = gripper;


    addCommands(
      // new WaitCommand(2),
      m_arm.PIDCommand(ArmConstants.kL1),
      m_elevator.PIDCommand(ElevatorConstants.L1),
      m_gripper.outtake()

    );
  }
}