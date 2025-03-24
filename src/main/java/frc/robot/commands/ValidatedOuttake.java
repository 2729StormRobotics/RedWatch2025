// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.html.InlineView;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ValidatedOuttake extends ParallelCommandGroup {
  /** Creates a new ValidatedOuttake. */
  private Gripper m_Gripper;
  private Arm m_Arm;
  private Elevator m_Elevator;
  private boolean m_inL4;
  public ValidatedOuttake(boolean inL4, Arm arm, Gripper gripper, Elevator elevator) {
    m_Gripper = gripper;
    m_Arm = arm;
    m_inL4 = inL4;
    m_Elevator = elevator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // SmartDashboard.putBoolean("commandl4", m_Elevator.getL4());
    inL4  = SmartDashboard.getBoolean("isl4", m_Elevator.getL4());
    
    SmartDashboard.putBoolean("commandl4", m_inL4);
    if (false) {
      addCommands(m_Gripper.outtake(),new SequentialCommandGroup(new WaitCommand(0.15), m_Arm.PIDCommand(ArmConstants.kSTOW)) );
    }
    else {
      addCommands(m_Gripper.outtake());
    }
  }
}
