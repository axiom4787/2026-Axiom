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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArm;

/** Represents the arm which deploys or stows the robot's fuel intake. */
public class IntakeArmSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(
      IntakeArm.MOTOR_ID, MotorType.kBrushless);

  // private final PIDController m_armPID = new PIDController(
  //   IntakeArm.ARM_P, 
  //   IntakeArm.ARM_I, 
  //   IntakeArm.ARM_D);

  // private boolean m_isDeployed = false; // True if the intake should be deployed, false if it should be stowed.
  private double m_motorPower = 0; // Represents the actual angle of the intake arm via encoder.

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
    
    m_motor.getEncoder().setPosition(0);
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
