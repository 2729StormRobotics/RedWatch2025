// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Vision;

public class AlgeAlign extends Command {
  /** Creates a new RotationAllign. */
  Vision m_vision; 
  Drive m_driveSubsystem;
  Joystick m_driverController;
  private final PIDController m_controller;
  private double m_turnError;
  private double m_turnPower;
  private final Joystick m_translator;

  public AlgeAlign(Joystick joystick, Drive drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = Vision.getInstance(); 
    m_driveSubsystem = drivetrain;
    // m_driverController = driverController;
    m_translator = joystick;
    m_controller = new PIDController(0.004, 0,0.002);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnError = 0;
    m_turnPower = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive the robot
    // m_driveSubsystem.drive(
    //   MathUtil.applyDeadband(m_driverController.getY()*OperatorConstants.translationMultiplier, OperatorConstants.kDriveDeadband),
    //   MathUtil.applyDeadband(m_driverController.getX()*OperatorConstants.translationMultiplier, OperatorConstants.kDriveDeadband),
    //   (m_controller.calculate(m_vision.getNoteXSkew()) + Math.copySign(Constants.VisionConstants.kSTurn, m_controller.calculate(m_vision.getNoteXSkew())) // Calculate P value
    //   ),
    //   false, true);

        CommandScheduler.getInstance()
        .schedule(
            DriveCommands.joystickDrive(
              m_driveSubsystem,
                () ->
                    MathUtil.applyDeadband(
                        -m_translator.getY() * OperatorConstants.translationMultiplier,
                        OperatorConstants.kDriveDeadband),
                () ->
                    MathUtil.applyDeadband(
                        -m_translator.getX() * OperatorConstants.translationMultiplier,
                        OperatorConstants.kDriveDeadband),
                () -> (m_turnPower)));
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return (Math.abs(turnError) < Constants.VisionConstants.kTolerance);
  }
}