// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Hubs;
import frc.robot.Constants.Swerve;
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
      .deadband(Swerve.CONTROLLER_DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  SwerveInputStream driveHubAligned = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .deadband(Swerve.CONTROLLER_DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .aim((DriverStation.getAlliance().isPresent()) && (DriverStation.getAlliance().get() == Alliance.Blue) ? Hubs.BLUE_HUB : Hubs.RED_HUB);

  // TODO: make only one input stream and use aimWhile instead of aim

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAngularVelocityCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedHubAligned = m_swerveSubsystem.driveFieldOriented(driveHubAligned);
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
    m_driverController.b().toggleOnTrue(driveFieldOrientedHubAligned); // press b to toggle regular/hub aligned driving
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
