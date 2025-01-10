// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.AutoCommandGroups;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/*
 * Makes the robot automatically align to nearest apriltag (while still being able to drive translationally)
 */
public class AutoAprilTagAlign extends Command {
  private final Vision m_vision; 
  private final Drivetrain m_drivetrain;
  private final PIDController m_controller;
  private double m_turnError;
  private double m_turnPower;
  /** Creates a new AprilTagAlign. */
  public AutoAprilTagAlign() {
    m_vision = Vision.getInstance();
    m_drivetrain = Drivetrain.getInstance();
    m_controller = new PIDController(0.004, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Values for PID Calculation
    m_turnError = 0;
    m_turnPower = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turnError = m_vision.getX(); // Horizontal angle away from target
    m_turnPower = m_controller.calculate(m_vision.getNoteXSkew()) + Math.copySign(Constants.VisionConstants.kSTurn, m_controller.calculate(m_vision.getNoteXSkew())); // Calculate P value
    if (Math.abs(m_vision.getX()) < 0.5) {
      m_turnPower = 0;
    }
    SmartDashboard.putNumber("turnError", m_turnError);
    // drive the robot
    m_drivetrain.drive(
      0,
      0,
      (m_turnPower),
      true, true);
      
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(
      0, 0, 0,
            true, true);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_turnError) < Constants.VisionConstants.aprilTagAlignTolerance;

  }
}