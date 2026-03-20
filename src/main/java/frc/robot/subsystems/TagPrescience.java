// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Limelight;

/** The state of knowing an effect before its cause has fully manifested. */
public class TagPrescience {
  /** A moment of clarity. */
  public record Revelation (boolean isManifest, Pose2d presence, double moment) {};

  /** Discerns a revelation. */
  public Revelation consult() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(Limelight.LL_NAME);
    double[] values = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    double moment = Timer.getFPGATimestamp();
    Pose2d presence = new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
    boolean isManifest = presence.getX() != 0.0 && presence.getY() != 0.0;
    return new Revelation(isManifest, presence, moment);
  }

  public record Projection (Pose2d projected) {};
  // The motion of the ball will be realtive to the chassis, so we can change where the target is
  //  by the motion of the chassis for the time the ball is in the air to green every time
  public Projection seeFuture(Pose2d target, double distance, ChassisSpeeds determination) {
    double regressionTimeOfFlight = distance; // TODO: TUNE!! In seconds. 
    
    Twist2d twistingIt = new Twist2d();
    // Moves in reverse relative to the robot. As the robot moves forward, the tags move closer to it
    twistingIt.dtheta = -determination.omegaRadiansPerSecond * regressionTimeOfFlight;
    twistingIt.dx = -determination.vxMetersPerSecond * regressionTimeOfFlight;
    twistingIt.dy = -determination.vyMetersPerSecond * regressionTimeOfFlight;

    return new Projection(target.exp(twistingIt));
  }
}
