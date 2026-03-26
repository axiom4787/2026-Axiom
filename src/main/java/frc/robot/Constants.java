package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;


public class Constants {
    public static final class Swerve {

        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;

        public static final PPHolonomicDriveController PP_CONTROLLER = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),  // translation PID
            new PIDConstants(5.0, 0.0, 0.0)); // rotation PID

    }
    public static final class IntakeRoller {
        public static final int MOTOR_ID = 12;

        public static final double INTAKE_SETPOINT = 200;

        public static final double INTAKE_S = 0.5;
        public static final double INTAKE_V = 0.021;
        public static final double INTAKE_P = 0.0;
        public static final double INTAKE_I = 0.0;
        public static final double INTAKE_D = 0.0;

        public static final double INTAKE_GEAR_RATIO = (6.0 / 7.0);

        public static final double INTAKE_CONVERSION_FACTOR = ((2.0 * Math.PI) / 60) * INTAKE_GEAR_RATIO;
    }
    public static final class IntakeArm {
        public static final int MOTOR_ID = 10;

        // public static final double UP_POWER = -0.1;
        // public static final double DOWN_POWER = 0.1;

        public static final double ARM_P = 0.0;
        public static final double ARM_I = 0.0;
        public static final double ARM_D = 0.0;

        public static final double STOW_SETPOINT = 0.15;
        public static final double DEPLOY_SETPOINT = -0.25;
    }

    public static final class Flywheel {
        public static final int RIGHT_MOTOR_ID = 6;
        public static final int LEFT_MOTOR_ID = 3;
        
        public static final double FLYWHEEL_S = 0.15;
        public static final double FLYWHEEL_V = 0.17;
        public static final double FLYWHEEL_P = 0.7;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0.01;

        public static final double FLYWHEEL_CONVERSION_FACTOR = (2.0 / Math.PI) / 60;

        public static final Translation2d FLYWHEEL_OFFSET = new Translation2d(0.0, 0.0); // TODO: check cad for correct offset

    }
    public static final class Conveyor {
        public static final int MOTOR_ID = 7;
        public static final double FEED_POWER = 0.75;
        public static final double INTAKE_POWER = 0.2;
        public static final double EJECT_POWER = -1;
    }
    public static final class Indexer {
        public static final int MOTOR_ID = 8;
        public static final double FEED_POWER = 0.75;
        public static final double EJECT_POWER = -1.0; 
    }
    public static final class Enlighten {
        public static final int LIGHT_ID = 0;
        public static final double IDLE_ACTIVE_COLOR_ID = 0.91; // Purple
        public static final double IDLE_INACTIVE_COLOR_ID = 0.95; // Grey
        public static final double INTAKING_COLOR_ID = -0.07; // Strobe Gold
        public static final double OUTTAKING_COLOR_ID = -0.11; // Strobe Red
        public static final double REVVING_HUB_COLOR_ID = 0.69; // Yellow
        public static final double REVVING_FEED_COLOR_ID = 0.65; // Orange
        public static final double READY_HUB_COLOR_ID = 0.73; // Lime Green
        public static final double READY_FEED_COLOR_ID = 0.75; // Dark Green
        public static final double SHOOT_COLOR_ID = -0.05; // Strobe White
    }

    // Time saved using this method: 12 seconds 
    // Time spent writing this method: 11.999 seconds 
    // Overall judgement: Innovative And Efficient Refactoring 
    private static Pose2d makeTarget(double x, double y)
    {
        return new Pose2d(new Translation2d(x, y), Rotation2d.kZero);
    }

    public static final class Targets {
        public static final double FIELD_LENGTH = 16.5405;
        public static final double FIELD_WIDTH = 8.0695;

        // Boundaries for zones of the field, used to determine which target to aim at
        public static final double BLUE_ALLIANCE_LINE_X = 4.4;
        public static final double RED_ALLIANCE_LINE_X = FIELD_LENGTH-4.4;
        public static final double CENTER_LINE_Y = FIELD_WIDTH/2;

        // Targets for the shooter to aim at
        public static final Pose2d BLUE_HUB = makeTarget(4.625, CENTER_LINE_Y);
        public static final Pose2d RED_HUB = makeTarget(FIELD_LENGTH-4.625, CENTER_LINE_Y);
        public static final Pose2d BLUE_SIM_START = makeTarget(3.57, CENTER_LINE_Y);
        public static final Pose2d RED_SIM_START = makeTarget(FIELD_LENGTH-3.57, CENTER_LINE_Y);
        public static final Pose2d BLUE_PASS_OUTPOST = makeTarget(0, 1);
        public static final Pose2d BLUE_PASS_DEPOT = makeTarget(0, FIELD_WIDTH-1);
        public static final Pose2d RED_PASS_OUTPOST = makeTarget(FIELD_LENGTH-0, FIELD_WIDTH-1);
        public static final Pose2d RED_PASS_DEPOT = makeTarget(FIELD_LENGTH-0, 1);

        // Auto starting positions, used to reset odometry at the beginning of auto
        public static final Pose2d BLUE_START_DEPOT = new Pose2d(4, 7.25, Rotation2d.fromDegrees(-79));
        public static final Pose2d BLUE_START_OUTPOST = new Pose2d(4, FIELD_WIDTH - 7.25, Rotation2d.fromDegrees(79));
        public static final Pose2d RED_START_OUTPOST = new Pose2d(FIELD_LENGTH-4, 7.25, Rotation2d.fromDegrees(-101));
        public static final Pose2d RED_START_DEPOT = new Pose2d(FIELD_LENGTH-4, FIELD_WIDTH - 7.25, Rotation2d.fromDegrees(101));
    }

    public static class Limelight {
        public static final String LL3RIGHT_NAME = "limelight-yamama";
        public static final String LL2LEFT_NAME = "limelight-zarqa"; // Look up Zarqa Al-Yamama for ball knowledge
    }
}
