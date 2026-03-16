package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;


public class Constants {
    public static final class Swerve {

        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;

    }
    public static final class IntakeRoller {
        public static final int MOTOR_ID = 14;

        public static final double INTAKE_POWER = 0.75;
        public static final double OUTTAKE_POWER = -0.75;
    }
    public static final class IntakeArm {
        public static final int MOTOR_ID = 10;

        // public static final double UP_POWER = -0.1;
        // public static final double DOWN_POWER = 0.1;

        public static final double ARM_P = 0.0;
        public static final double ARM_I = 0.0;
        public static final double ARM_D = 0.0;

        public static final double STOW_SETPOINT = 0.0;
        public static final double DEPLOY_SETPOINT = 0.0;

        // TODO: Tune Intake Arm PID
    }

    public static final class Flywheel {
        public static final int RIGHT_MOTOR_ID = 11;
        public static final int LEFT_MOTOR_ID = 12;
        
        public static final double FLYWHEEL_S = 0.15;
        public static final double FLYWHEEL_V = 0.17;
        public static final double FLYWHEEL_P = 0.7;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0.01;

        public static final double FLYWHEEL_CONVERSION_FACTOR = (2.0 / Math.PI) / 60;
    }
    public static final class Conveyor {
        public static final int MOTOR_ID = 9;
        public static final double FEED_POWER = -0.25;
        public static final double EJECT_POWER = 0.25;
    }
    public static final class Indexer {
        public static final int MOTOR_ID = 15;
        public static final double FEED_POWER = -1.0;
        public static final double EJECT_POWER = 1.0;
    }

    private static Pose2d makeTarget(double x, double y)
    {
        return new Pose2d(new Translation2d(x, y), Rotation2d.kZero);
    }

    public static final class Targets {
        public static final double FIELD_LENGTH = 16.5405;
        public static final double FIELD_WIDTH = 8.0695;

        public static final double BLUE_ALLIANCE_LINE_X = 4.4;
        public static final double RED_ALLIANCE_LINE_X = FIELD_LENGTH-4.4;
        public static final double CENTER_LINE_Y = FIELD_WIDTH/2;

        public static final Pose2d BLUE_HUB = makeTarget(4.625, CENTER_LINE_Y);
        public static final Pose2d RED_HUB = makeTarget(FIELD_LENGTH-4.625, CENTER_LINE_Y);
        public static final Pose2d BLUE_PASS_OUTPOST = makeTarget(1.5, 1.5);
        public static final Pose2d BLUE_PASS_DEPOT = makeTarget(1.5, FIELD_WIDTH-1.5);
        public static final Pose2d RED_PASS_OUTPOST = makeTarget(FIELD_LENGTH-1.5, FIELD_WIDTH-1.5);
        public static final Pose2d RED_PASS_DEPOT = makeTarget(FIELD_LENGTH-1.5, 1.5);
    }

    public static class Limelight {
        public static final String LL_NAME = "limelight-zarqa"; // Only owls will understand

        // Ignorance is bliss.
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
    }
}
