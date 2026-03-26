// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArm;

/** Represents the arm which deploys or stows the robot's fuel intake. */
public class IntakeArmSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(
      IntakeArm.MOTOR_ID, MotorType.kBrushless);

  private double m_motorPower = 0;

  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {
    m_motor.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(60);

    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPower(double motorPower)
  {
    m_motorPower = motorPower;
  }

  @Override
  public void periodic() {
    m_motor.set(m_motorPower);
    SmartDashboard.putNumber("Intake/Arm Power", m_motorPower);
  }
}
