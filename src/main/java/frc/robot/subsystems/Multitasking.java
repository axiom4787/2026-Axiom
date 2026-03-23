// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Multitasking {
    
    // --- Hardware & Field Constants ---
    private static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches
    private static final double SHOOTER_ANGLE_RADS = Math.toRadians(72);
    private static final double SHOOTER_EFFICIENCY_K = 0.45; // Tune this!
    
    // --- NEW: Shooter Offset ---
    // The physical location of the shooter's exit point relative to the center of the robot.
    // Example: -0.2 meters X (backwards), 0.0 meters Y (centered left/right)
    private static final Translation2d ROBOT_TO_SHOOTER = new Translation2d(Units.inchesToMeters(9.7), 0.0);

    /**
     * YOUR REGRESSION: Put your calculated root regression formula here.
     */
    private static double getRequiredRPM(double distance) {
        // Example placeholder: replace with your actual math!
        return 30*Math.PI*3.777704*Math.pow(Units.metersToInches(distance), 0.455459);
    }

    /**
     * Calculates the time of flight based on the physical properties of the shot.
     */
    private static double calculateTimeOfFlight(double distanceToTarget) {
        double targetRPM = getRequiredRPM(distanceToTarget); 
        double surfaceVelocity = targetRPM * ((Math.PI * WHEEL_DIAMETER_METERS) / 60.0);
        double exitVelocity = surfaceVelocity * SHOOTER_EFFICIENCY_K;
        
        if (exitVelocity <= 0.01) return 0.0; // Prevent divide-by-zero
        
        double horizontalVelocity = exitVelocity * Math.cos(SHOOTER_ANGLE_RADS);
        return distanceToTarget / horizontalVelocity;
    }

    /**
     * NEW: Calculates the tangential velocity vector of the shooter based on the robot's spin.
     */
    private static Translation2d getTangentialVelocity(Rotation2d robotHeading, double omegaRadiansPerSecond) {
        // 1. Find the field-relative offset of the shooter based on current heading
        Translation2d fieldRelativeOffset = ROBOT_TO_SHOOTER.rotateBy(robotHeading);
        
        // 2. Calculate tangential velocity (Cross product of omega and radius vector)
        // V_x = -omega * r_y
        // V_y = omega * r_x
        return new Translation2d(
            -fieldRelativeOffset.getY() * omegaRadiansPerSecond,
            fieldRelativeOffset.getX() * omegaRadiansPerSecond
        );
    }

    /**
     * The Iterative Loop (Now requires Pose2d instead of Translation2d to get current rotation).
     * * @param robotPose Current estimated Pose2d of the robot from Odometry.
     * @param fieldRelativeSpeeds Current field-relative velocities of the chassis.
     * @return Pose to aim at
     */
    public static Pose2d calculate(Pose2d targetPose, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        // Extract standard linear velocity
        Translation2d linearVelocity = new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, 
            fieldRelativeSpeeds.vyMetersPerSecond
        );
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;

        Translation2d robotTranslation = robotPose.getTranslation();
        
        // Start our heading guess at our current heading
        Rotation2d headingGuess = robotPose.getRotation();

        // --- ITERATION 1: Initial Guess ---
        double distance = robotTranslation.getDistance(targetPose.getTranslation());
        double timeOfFlight = calculateTimeOfFlight(distance);

        // Combine linear and tangential velocity based on our heading guess
        Translation2d tangentialVelocity = getTangentialVelocity(headingGuess, omega);
        Translation2d totalShooterVelocity = linearVelocity.plus(tangentialVelocity);

        // Offset the HUB by the total velocity the ball inherits
        Translation2d virtualTarget = targetPose.getTranslation().minus(totalShooterVelocity.times(timeOfFlight));
        
        // Update our heading guess to point at the newly found virtual target
        headingGuess = virtualTarget.minus(robotTranslation).getAngle();

        // --- ITERATION 2: Convergence ---
        distance = robotTranslation.getDistance(virtualTarget);
        timeOfFlight = calculateTimeOfFlight(distance);

        // Recalculate tangential velocity using our NEW, much more accurate heading guess
        tangentialVelocity = getTangentialVelocity(headingGuess, omega);
        totalShooterVelocity = linearVelocity.plus(tangentialVelocity);

        // --- FINAL CALCULATION ---
        virtualTarget = targetPose.getTranslation().minus(totalShooterVelocity.times(timeOfFlight));

        return new Pose2d(virtualTarget, Rotation2d.kZero);
    }
}
