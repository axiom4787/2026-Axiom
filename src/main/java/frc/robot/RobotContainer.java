// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem((new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/robot")));
  private final CommandXboxController m_driverController = new CommandXboxController(0); 

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                              () -> m_driverController.getLeftY() * -1,
                                                              () -> m_driverController.getLeftX() * -1)
                                                          .withControllerRotationAxis(m_driverController::getRightX)
                                                          .deadband(Constants.Swerve.CONTROLLER_DEADBAND)
                                                          .scaleTranslation(0.8)
                                                          .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAngularVelocityCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
