// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Hubs;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem((new File(Filesystem.getDeployDirectory(),
      "swerve/robot")));
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private boolean aiming = false;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(Swerve.CONTROLLER_DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .aim((DriverStation.getAlliance().isPresent()) && (DriverStation.getAlliance().get() == Alliance.Blue) ? Hubs.BLUE_HUB : Hubs.RED_HUB)
      .aimWhile(this::isAiming);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAngularVelocityCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
    m_driverController.b().onTrue(new InstantCommand(() -> {
      aiming = !aiming;
    })); // press b to toggle regular/hub aligned driving
    m_driverController.a().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro, m_swerveSubsystem)); // TODO: determine if zeroGyroWithAlliance is better
    m_driverController.leftStick().whileTrue(new RunCommand(m_swerveSubsystem::lock, m_swerveSubsystem));
  }

  private boolean isAiming() {
    return aiming;
  }


  public Command getAutonomousCommand() {
    return null;
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerveSubsystem;
  }
}
