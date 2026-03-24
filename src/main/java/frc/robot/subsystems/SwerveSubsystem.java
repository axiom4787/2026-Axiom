// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.Flywheel;
import frc.robot.Constants.Limelight;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Targets;
import frc.robot.subsystems.TagPrescience.Revelation;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;

  /**
   * The unblinking eye.
   */
  private final TagPrescience LL3Left;

  private AimMode m_aimMode = AimMode.HUB;

  private Pose2d m_aimTarget = Pose2d.kZero;

  // private Pose2d m_virtualTarget = new Pose2d();

  private double m_targetDist = 0;

  // private double m_virtualDist = 0;

  private Field2d field = new Field2d();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    LL3Left = new TagPrescience(Limelight.LL3LEFT_NAME);

    SmartDashboard.putData("Full Field", field);

    Pose2d startingPose = Pose2d.kZero;

    // if (RobotBase.isSimulation())
    //   startingPose = isRedAlliance() ? Targets.RED_SIM_START : Targets.BLUE_SIM_START;
    // boolean blueAlliance = false;
    // Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
    //     Meter.of(4)),
    //     Rotation2d.fromDegrees(0))
    //     : new Pose2d(new Translation2d(Meter.of(16),
    //         Meter.of(4)),
    //         Rotation2d.fromDegrees(180));
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Swerve.MAX_SPEED, startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    
    pathplannerInit();
  }

  @Override
  public void periodic() {
    // Update pose estimator based on vision results
    Revelation revelation = LL3Left.consult(getHeading().getDegrees(), getPitch().getDegrees(), getRoll().getDegrees());

    SmartDashboard.putBoolean("Clarity Provided", revelation.isManifest());
    
    if (revelation.isManifest() && DriverStation.isTeleopEnabled()) {
      swerveDrive.addVisionMeasurement(revelation.presence().pose, revelation.presence().timestampSeconds, revelation.trust());
    }

    swerveDrive.updateOdometry();

    // Calculate which target to aim at, target type, and target distance for auto-aim purposes.
    Pose2d pose = getPose();

    if (isRedAlliance()) {
      if (pose.getX() > Targets.RED_ALLIANCE_LINE_X) {
        m_aimMode = AimMode.HUB;
        m_aimTarget = Targets.RED_HUB;
      } else {
        m_aimMode = AimMode.FEED;
        m_aimTarget = pose.getY() > Targets.CENTER_LINE_Y ? Targets.RED_PASS_OUTPOST : Targets.RED_PASS_DEPOT;
      }
    } else {
      if (pose.getX() < Targets.BLUE_ALLIANCE_LINE_X) {
        m_aimMode = AimMode.HUB;
        m_aimTarget = Targets.BLUE_HUB;
      } else {
        m_aimMode = AimMode.FEED;
        m_aimTarget = pose.getY() > Targets.CENTER_LINE_Y ? Targets.BLUE_PASS_DEPOT : Targets.BLUE_PASS_OUTPOST;
      }
    }

    m_targetDist = Math.abs(pose.getTranslation().getDistance(m_aimTarget.getTranslation()));

    // TODO: calculate alpha and v_0 based on regressions and make them private class variables

    // m_virtualTarget = Multitasking.calculate(m_aimTarget, pose, getFieldVelocity());
    // 1. Get Field-Relative Robot Velocity
    // We need to know how the robot is moving relative to the FLOOR, not itself.
    // ChassisSpeeds fieldSpeeds = getFieldVelocity();

    // // 2. Calculate TOF (Ensure distance is in INCHES for the regression)
    // double distMeters = m_aimTarget.getTranslation().getDistance(getPose().getTranslation());
    // double distInches = Units.metersToInches(distMeters);
    // double tof = -0.000000425 * Math.pow(distInches, 2) + 0.00767 * distInches + 0.413;

    // SmartDashboard.putNumber("Shooter/Ball TOF", tof);

    // // 3. Calculate the "Compensation Vector" 
    // // This is how far the ball "drifts" because of the robot's movement
    // Translation2d ballDrift = new Translation2d(
    //     fieldSpeeds.vxMetersPerSecond * tof,
    //     fieldSpeeds.vyMetersPerSecond * tof
    // );

    // // 4. Create the Virtual Target
    // // Subtract the drift from the goal's real location
    // Translation2d virtualTargetTranslation = m_aimTarget.getTranslation().minus(ballDrift);

    // // 5. Update your Turret/Drive to aim at THIS Translation2d
    // m_virtualTarget = new Pose2d(virtualTargetTranslation, new Rotation2d());    

    // // 1. Convert current distance to inches for the regression
    // double currentDistInches = Units.metersToInches(m_targetDist);
    // double tof = -0.000000425 * Math.pow(currentDistInches, 2) + 0.00767 * currentDistInches + 0.413;

    // // 2. Calculate velocity at shooter
    // ChassisSpeeds robotVel = getRobotVelocity();
    // double shooterVx = robotVel.vxMetersPerSecond - (robotVel.omegaRadiansPerSecond * Flywheel.FLYWHEEL_OFFSET.getY());
    // double shooterVy = robotVel.vyMetersPerSecond + (robotVel.omegaRadiansPerSecond * Flywheel.FLYWHEEL_OFFSET.getX());

    // // 3. Iteration Step: Find virtual target once to refine TOF
    // Twist2d twist = new Twist2d(-shooterVx * tof, -shooterVy * tof, -robotVel.omegaRadiansPerSecond * tof);
    // Pose2d intermediateTarget = m_aimTarget.exp(twist);

    // // 4. Update TOF based on the virtual distance
    // double virtualDistInches = Units.metersToInches(Math.abs(pose.getTranslation().getDistance(intermediateTarget.getTranslation())));
    // double refinedTof = -0.000000425 * Math.pow(virtualDistInches, 2) + 0.00767 * virtualDistInches + 0.413;

    // // 5. Final Virtual Target
    // Twist2d finalTwist = new Twist2d(-shooterVx * refinedTof, -shooterVy * refinedTof, -robotVel.omegaRadiansPerSecond * refinedTof);
    // m_virtualTarget = m_aimTarget.exp(finalTwist);

    // m_virtualDist = Math.abs(pose.getTranslation().getDistance(m_virtualTarget.getTranslation()));

    field.setRobotPose(pose);
    field.getObject("target").setPose(m_aimTarget);
    // field.getObject("vtarget").setPose(m_virtualTarget);
    SmartDashboard.putString("Shooter/Aim Mode", m_aimMode.toString());
    // SmartDashboard.putNumber("Shooter/Target Distance", m_targetDist);
    // SmartDashboard.putNumber("Shooter/VTarget Distance", m_virtualDist);
  }

  @Override
  public void simulationPeriodic() {
  }

  public void pathplannerInit() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry,
      this::getRobotVelocity,
      (speeds, feedforwards) -> drive(speeds),
      Swerve.PP_CONTROLLER,
      config,
      this::isRedAlliance,
      this);
  }

  /**
   * Gets where the robot should aim based on its location on the field.
   */
  public Pose2d getTarget() {
    return m_aimTarget; 
  }

  /**
   * Gets how far the calculated aim target is from the robot, in meters.
   */
  public double getTargetDistance() {
    return m_targetDist;
  }

  public Rotation2d getTargetOffset()
  {
    return Rotation2d.kZero; // TODO: return alpha based on regression
  }

  public double getShootSpeed() {
    return 0.0; // TODO: return flywheel speed based on v_0 regression
  }

  /**
   * Gets whether the calculated aim target is a hub or a feed zone, based on the latest call to calculateTarget.
   */
  public AimMode getAimMode() {
    return m_aimMode;
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command
   * ends.
   *
   * @return a Command that tells the robot to drive forward until the command
   *         ends
   */
  public Command driveForward() {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveBackward() {
    return run(() -> {
      swerveDrive.drive(new Translation2d(-0.5, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
   * object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother
   *                         controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother
   *                         controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   *                         smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
    // resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is
   *         available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.Swerve.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.Swerve.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Gets the current roll angle of the robot, as reported by the imu.
   * 
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getRoll() {
    return swerveDrive.getRoll();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public enum AimMode {
    HUB,
    FEED
  }
}