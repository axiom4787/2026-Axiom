package frc.robot;

public class Constants {
    public static final class Swerve {

        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;

    }
    public static final class IntakeRoller {
        public static final int MOTOR_ID = 14;
        public static final double INTAKE_POWER = 0.4;
    }
    public static final class IntakeArm {
        public static final int MOTOR_ID = 15;
        public static final double UP_POWER = 0.3;
        public static final double DOWN_POWER = -0.3;
    }

    public static final class Flywheel {
        public static final int FLYWHEEL_SOLO_MOTOR_ID = 11;
        public static final int FLYWHEEL_DUO_UPPER_MOTOR_ID = 12;
        public static final int FLYWHEEL_DUO_LOWER_MOTOR_ID = 13;
        
        public static final double FLYWHEEL_S = 0.15;
        public static final double FLYWHEEL_V = 0.17;
        public static final double FLYWHEEL_P = 0.7;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0.01;

        public static final double FLYWHEEL_CONVERSION_FACTOR = (2.0 / Math.PI) / 60;
    
        public static final double FLYWHEEL_SETPOINT = 25;
    }
    public static final class Conveyor {
        public static final int MOTOR_ID = 9;
        public static final double POWER = 1.0;
    }
    public static final class Indexer {
        public static final int MOTOR_ID = 10;
        public static final double POWER = -1.0;
    }
}
