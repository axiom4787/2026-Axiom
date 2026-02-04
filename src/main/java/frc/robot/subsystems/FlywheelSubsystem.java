// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flywheel;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */
  private final SparkMax m_flywheelMotor = new SparkMax(Flywheel.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

  private final PIDController m_flywheelPID = new PIDController(
    Flywheel.FLYWHEEL_P,
    Flywheel.FLYWHEEL_I,
    Flywheel.FLYWHEEL_D
  );

  private final SimpleMotorFeedforward m_flywheelFF = new SimpleMotorFeedforward(
    Flywheel.FLYWHEEL_S,
    Flywheel.FLYWHEEL_V
  );
  
  private double m_desiredSpeed = 0;

  public FlywheelSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    
    config
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40)
      .encoder
        .velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_flywheelMotor.configure(
      config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
                            
                              
                            
  }

  public void setDesiredSpeed(double desiredSpeed) {
    m_desiredSpeed = desiredSpeed;
  }

  @Override
  public void periodic() {
    double currentSpeed = m_flywheelMotor
                            .getEncoder()
                            .getVelocity();

    SmartDashboard.putNumber("Shooter/Desired Speed", m_desiredSpeed);
    SmartDashboard.putNumber("Shooter/Current Speed", currentSpeed);
    SmartDashboard.putNumber(
      "Shooter/Applied Voltage",
      m_flywheelMotor.getBusVoltage()
    );
    SmartDashboard.putData("Shooter/PID", m_flywheelPID);

    double feedforward = m_flywheelFF
                          .calculate(m_desiredSpeed);
    
    double feedback    = m_flywheelPID
                          .calculate(m_desiredSpeed, currentSpeed);

    m_flywheelMotor.setVoltage(feedback + feedforward);

  }
}

