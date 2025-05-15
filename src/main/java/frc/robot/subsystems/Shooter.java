package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final SparkMax leftShooterMotor;
  private final SparkMax rightShooterMotor;
  private final SparkMaxConfig shooterConfig;

  public Shooter() {
    leftShooterMotor = new SparkMax(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightShooterMotor = new SparkMax(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterConfig = new SparkMaxConfig();
    configureMotors();
  }

  private void configureMotors() {
    SparkBaseConfig baseConfig = new SparkMaxConfig()
        .voltageCompensation(12.0)
        .smartCurrentLimit(40)
        .idleMode(ShooterConstants.SHOOTER_IDLE_MODE);

    leftShooterMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooterMotor.configure(baseConfig.follow(leftShooterMotor), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Invert right motor to ensure both motors spin in the same direction
    rightShooterMotor.setInverted(true);
  }

  public void shoot(double speed) {
    // Apply speed limits
    speed = Math.max(-ShooterConstants.MAX_SHOOTER_SPEED, 
                    Math.min(ShooterConstants.MAX_SHOOTER_SPEED, speed));
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void stop() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 