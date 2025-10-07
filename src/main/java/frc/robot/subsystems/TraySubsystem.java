package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TrayConstants;

public class TraySubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMax feedMotor;

  public TraySubsystem() {
    motor = new SparkMax(TrayConstants.MotorId, MotorType.kBrushless);
    feedMotor = new SparkMax(TrayConstants.FeedMotorId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(TrayConstants.MotorId, true);
    feedMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Add a SmartDashboard slider for manual control
    SmartDashboard.putNumber("Coral Motor Speed", 0.0);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
    SmartDashboard.putNumber("Coral Motor Applied Output", motor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // Optional: allow manual override from dashboard
    double manualSpeed = SmartDashboard.getNumber("Coral Motor Speed", 0.0);
    if (manualSpeed != 0.0) {
      motor.set(manualSpeed);
    }
  }
}
