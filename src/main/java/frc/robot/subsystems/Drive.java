package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  private final SparkMax leftMotor1;
  private final SparkMax leftMotor2;
  private final SparkMax rightMotor1;
  private final SparkMax rightMotor2;

  private final SparkMaxConfig left1Config;
  private final SparkMaxConfig left2Config;
  private final SparkMaxConfig right1Config;
  private final SparkMaxConfig right2Config;

  public Drive() {
    // Initialize motors
    leftMotor1 = new SparkMax(DriveConstants.LEFT_MOTOR_1_ID, MotorType.kBrushless);
    leftMotor2 = new SparkMax(DriveConstants.LEFT_MOTOR_2_ID, MotorType.kBrushless);
    rightMotor1 = new SparkMax(DriveConstants.RIGHT_MOTOR_1_ID, MotorType.kBrushless);
    rightMotor2 = new SparkMax(DriveConstants.RIGHT_MOTOR_2_ID, MotorType.kBrushless);

    // Initialize configs
    left1Config = new SparkMaxConfig();
    left2Config = new SparkMaxConfig();
    right1Config = new SparkMaxConfig();
    right2Config = new SparkMaxConfig();

    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    // Configure all motors with the same settings
    SparkBaseConfig baseConfig = new SparkMaxConfig()
        .voltageCompensation(12.0)
        .smartCurrentLimit(40)
        .idleMode(DriveConstants.DRIVE_IDLE_MODE);

    // Apply base config to all motors
    leftMotor1.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor2.configure(baseConfig.follow(leftMotor1), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor1.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor2.configure(baseConfig.follow(rightMotor1), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set followers
    // leftMotor2.follow(leftMotor1, true);  // true for inverted following
    // rightMotor2.follow(rightMotor1, true);  // true for inverted following

    // Invert right side
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    // Apply speed limits
    leftSpeed = Math.max(-DriveConstants.MAX_SPEED, Math.min(DriveConstants.MAX_SPEED, leftSpeed));
    rightSpeed = Math.max(-DriveConstants.MAX_SPEED, Math.min(DriveConstants.MAX_SPEED, rightSpeed));

    // Set motor speeds
    leftMotor1.set(leftSpeed);
    rightMotor1.set(rightSpeed);
  }

  public void stop() {
    leftMotor1.set(0);
    rightMotor1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 