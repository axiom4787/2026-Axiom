// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArm;

/** Represents the arm which deploys or stows the robot's fuel intake. */
public class IntakeArmSubsystem extends SubsystemBase {
  private final SparkFlex m_motor = new SparkFlex(
      IntakeArm.MOTOR_ID, MotorType.kBrushless);

  private final PIDController m_armPID = new PIDController(
    IntakeArm.ARM_P, 
    IntakeArm.ARM_I, 
    IntakeArm.ARM_D);

  private boolean m_isDeployed = false; // True if the intake should be deployed, false if it should be stowed.
  private double m_currentAngle = IntakeArm.STOW_SETPOINT; // Represents the actual angle of the intake arm via encoder.

  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {
    m_motor.setCANTimeout(250);

    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Sets the desired state of the intake arm to deploy or stow.
   * @param deploy True for deploying, false for stowing.
   */
  public void setDeployed(boolean deploy) {
    m_isDeployed = deploy;
  }

  /**
   * Gets whether the intake arm is deployed (true) or stowed (false)
   */
  public boolean isDeployed() {
    return m_isDeployed;
  }

  @Override
  public void periodic() {
    double setpoint = m_isDeployed ? IntakeArm.DEPLOY_SETPOINT : IntakeArm.STOW_SETPOINT;
    m_currentAngle = m_motor.getEncoder().getPosition();

    SmartDashboard.putBoolean("Intake/Deployed", m_isDeployed);
    SmartDashboard.putNumber("Intake/Arm Angle", m_currentAngle);
    SmartDashboard.putData("Intake/Arm PID", m_armPID);

    double calculation = m_armPID.calculate(m_currentAngle, setpoint);
    m_motor.set(calculation);
  }
}
