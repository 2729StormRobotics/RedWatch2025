// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Turret turret;
  private final Loader loader;
  private final Pivot pivot;
  private final Shooter shooter;

  // Controllers
  private final XboxController driverController;
  private final XboxController operatorController;

  public RobotContainer() {
    // Initialize subsystems
    drive = new Drive();
    turret = new Turret();
    loader = new Loader();
    pivot = new Pivot();
    shooter = new Shooter();

    // Initialize controllers
    driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    // Configure button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Driver controls
    // Tank drive
    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.tankDrive(
                -driverController.getLeftY() * 0.1,
                -driverController.getRightY() * 0.1),
            drive));

    // Turret controls
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> turret.rotate(0.1), turret));
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> turret.rotate(-0.1), turret));

    // Operator controls
    // Loader controls
    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> loader.load(0.1), loader));
    new JoystickButton(operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> loader.load(-0.1), loader));

    // Pivot controls
    new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> pivot.moveToIntakePosition(), pivot));
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> pivot.moveToZeroPosition(), pivot));

    // Shooter controls
    new JoystickButton(operatorController, XboxController.Button.kStart.value)
        .whileTrue(new RunCommand(() -> shooter.shoot(0.1), shooter));
    new JoystickButton(operatorController, XboxController.Button.kBack.value)
        .whileTrue(new RunCommand(() -> shooter.shoot(-0.1), shooter));

    // Stop commands
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> {
          drive.stop();
          turret.stop();
        }));

    new JoystickButton(operatorController, XboxController.Button.kLeftStick.value)
        .onTrue(new InstantCommand(() -> {
          loader.stop();
          pivot.stop();
          shooter.stop();
        }));
  }

  public Command getAutonomousCommand() {
    // Return a default auto command
    return new InstantCommand();
  }
}
