// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Targets;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem((new File(Filesystem.getDeployDirectory(),
      "swerve/robot")));
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  // Base input stream with no rotation control
  SwerveInputStream baseStream = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .deadband(Swerve.CONTROLLER_DEADBAND)
      .allianceRelativeControl(true);

  // Input stream and command to drive robot with right joystick as rotation control
  SwerveInputStream driveAngularVelocity = baseStream.copy()
    .withControllerRotationAxis(m_driverController::getRightX);
  Command driveAngVelCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
  
  // Same as previous, but with slowed inputs for precision
  Command driveAngVelSlowCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity.copy()
    .scaleTranslation(0.5)
    .scaleRotation(0.5));
  
  // Command to drive robot while aiming at a target
  Command driveWithAimCommand = m_swerveSubsystem.driveFieldOriented(baseStream.copy()
    .scaleTranslation(0.5) // TODO: Determine if slow drive or regular drive is ideal for aiming mode
    .aim(m_swerveSubsystem::calculateTarget)
    .aimWhile(true));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // TODO: Map drive controls to FTC controller
    // Drive controls
    // B: Aim at selected target while held.
    // Y: Drive slowly while held.
    // Left Stick: Lock wheels while held.
    // A: Reset gyro heading
    
    m_driverController.b().whileTrue(driveWithAimCommand);
    m_driverController.y().whileTrue(driveAngVelSlowCommand);
    m_driverController.leftStick().whileTrue(new RunCommand(m_swerveSubsystem::lock, m_swerveSubsystem));
    m_driverController.a().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro, m_swerveSubsystem)); // TODO: determine if zeroGyroWithAlliance is better

    m_swerveSubsystem.setDefaultCommand(driveAngVelCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerveSubsystem;
  }
}
