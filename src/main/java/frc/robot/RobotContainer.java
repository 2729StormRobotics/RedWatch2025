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

import static frc.robot.util.drive.DriveControls.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Gripper.Intake;
import frc.robot.commands.Gripper.Outtake;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.Gripper.GripperIO;
import frc.robot.subsystems.Gripper.GripperIOSim;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.GripperIOSparkMax;
import frc.robot.util.drive.DriveControls;
/*
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
Comment for magic box testing purposes*/

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Gripper m_gripper;

  private final XboxController controller = new XboxController(2);


  private boolean brakeMode = true;

  // LEDs
  private final BlinkinLEDController ledController = BlinkinLEDController.getInstance();

  // Controller
  private final CommandJoystick m_translator = new CommandJoystick(1);
  private final CommandJoystick m_rotator = new CommandJoystick(0);
  private final CommandXboxController m_weaponsController = new CommandXboxController(0);

  //   private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  /*private LoggedDashboardChooser<Command> autoChooser;
  private LoggedDashboardBoolean brakeModeDashboard =
      new LoggedDashboardBoolean("Brake Mode", true);
  private LoggedDashboardBoolean setStartPosition =
      new LoggedDashboardBoolean("Set Start Position", false);
  Comment for magic box testing purposes*/
  // Field
  private final Field2d field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOReal(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        m_gripper = new Gripper(new GripperIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_gripper = new Gripper(new GripperIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_gripper = new Gripper(new GripperIO() {});
        break;
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    System.out.println("[Init] Setting up Path Planner Logging");

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
          //Logger.recordOutput("PathPlanner/RobotPose", pose); comment for magic box testing
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
          //Logger.recordOutput("PathPlanner/TargetPose", pose); comment for magic box testing
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
          //Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0])); comment for magic box testing
        });

    // Set up auto routines
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
    //         .withTimeout(5.0));
    //autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()); comment for magic box testing

    // Set up SysId routines
    /*autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        comment for magic box testing*/
    // Configure the button bindings
    configureButtonBindings();

    // Set up auto routines
    System.out.println("[Init] Setting up Logged Auto Chooser");
    //autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()); comment for magic box testing
  }
  // zero gyro
  public void reset() {
    drive.resetYaw();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Gripper
    new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand(() -> {new Intake(m_gripper);}));
    new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(() -> {new Outtake(m_gripper);}));
    new JoystickButton(controller, Button.kY.value).onTrue(new InstantCommand(() -> {m_gripper.stop();}));
    
    // default subsystem commands
    DriveControls.configureControls();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));

    RESET_GYRO.onTrue(
        new InstantCommand(
            () -> {
              drive.resetYaw();
            },
            drive));
    // RESET_GYRO.whileTrue(
    //     new InstantCommand(
    //         () -> {
    //           drive.resetYaw();
    //         },
    //         null));

    QUASISTATIC_FORWARD.whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    QUASISTATIC_REVERSE.whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
    DYNAMIC_FORWARD.whileTrue(drive.sysIdDynamic(Direction.kForward));
    DYNAMIC_REVERSE.whileTrue(drive.sysIdDynamic(Direction.kReverse));
    // Drive setting commands
    // DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    // DRIVE_STOP.onTrue(
    //     new InstantCommand(
    //         () -> {
    //           drive.stopWithX();
    //           drive.resetYaw();
    //         },
    //         drive));

    // DRIVE_HOLD_STOP.onTrue(
    //     new InstantCommand(
    //         () -> {
    //           drive.stopWithX();
    //         },
    //         drive));

    // // Drive Modes
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  comment for magic box testing*/
}
