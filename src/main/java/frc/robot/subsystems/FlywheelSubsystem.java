// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flywheel;

/** Represents the flywheels that launch fuel out of the robot. */
public class FlywheelSubsystem extends SubsystemBase {
  private final SparkFlex m_rightMotor = new SparkFlex(Flywheel.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_leftMotor = new SparkFlex(Flywheel.LEFT_MOTOR_ID,
      MotorType.kBrushless);

  private final PIDController m_flywheelPID = new PIDController(
      Flywheel.FLYWHEEL_P,
      Flywheel.FLYWHEEL_I,
      Flywheel.FLYWHEEL_D);

  private final SimpleMotorFeedforward m_flywheelFF = new SimpleMotorFeedforward(
      Flywheel.FLYWHEEL_S,
      Flywheel.FLYWHEEL_V);

  private double m_desiredSpeed = 0;

  private double m_currentSpeed = 0;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40).encoder
        .velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_rightMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig config2 = new SparkMaxConfig();

    config2
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .follow(m_rightMotor, true).encoder.velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_leftMotor.configure(
        config2,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter/Setpoint", 0.0);
  }

  /**
   * Sets the desired speed (setpoint) of the flywheels. 
   * A combination of feedforward and PID control will bring the flywheels to the desired speed over time.
   * @param desiredSpeed The speed setpoint, in radians per second.
   */
  public void setDesiredSpeed(double desiredSpeed) {
    m_desiredSpeed = desiredSpeed;
  }

  /**
   * Whether or not the flywheels are at the correct speed (i.e. ready to fire a shot)
   * @return True if the current speed is within 0.5 radians per second of the speed setpoint, false otherwise.
   */
  public boolean atSpeed() {
    return Math.abs(m_desiredSpeed - m_currentSpeed) < 0.5;
  }

  @Override
  public void periodic() {
    // When the shooter is disabled we don't want it to quickly come to a stop with PID.
    // Rather, we want it to coast slowly to a stop to avoid damaging the chains.
    if (m_desiredSpeed == 0) {
      m_rightMotor.setVoltage(0);
      return;
    }

    // Regular PID/Feedforward logic
    m_currentSpeed = m_rightMotor.getEncoder().getVelocity();

    SmartDashboard.putNumber("Shooter/Desired Speed", m_desiredSpeed);
    SmartDashboard.putNumber("Shooter/Current Speed", m_currentSpeed);
    SmartDashboard.putData("Shooter/PID", m_flywheelPID);

    double feedforward = m_flywheelFF.calculate(m_desiredSpeed);

    double feedback = m_flywheelPID.calculate(m_currentSpeed, m_desiredSpeed);

    SmartDashboard.putNumber("Shooter/Feedforward", feedforward);
    SmartDashboard.putNumber("Shooter/Feedback", feedback);

    m_rightMotor.setVoltage(feedback + feedforward);
  }
}
