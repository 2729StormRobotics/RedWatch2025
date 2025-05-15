package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  private final SparkMax turretMotor;
  private final SparkMaxConfig turretConfig;

  public Turret() {
    turretMotor = new SparkMax(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    turretConfig = new SparkMaxConfig();
    configureMotor();
  }

  private void configureMotor() {
    SparkBaseConfig baseConfig = new SparkMaxConfig()
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .idleMode(TurretConstants.TURRET_IDLE_MODE);

    turretMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void rotate(double speed) {
    // Apply speed limits
    speed = Math.max(-TurretConstants.MAX_TURRET_SPEED, 
                    Math.min(TurretConstants.MAX_TURRET_SPEED, speed));
    turretMotor.set(speed);
  }

  public void stop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 