// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRoller;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(
      IntakeRoller.MOTOR_ID, MotorType.kBrushless);

  private double m_motorPower = 0.0;

  public IntakeSubsystem() {
    m_motor.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setIndexerPower(double motorPower) {
    m_motorPower = motorPower;
    m_motor.set(m_motorPower);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Roller Motor Power", m_motorPower);
  }
}
