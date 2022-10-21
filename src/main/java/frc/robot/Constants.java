package frc.robot;

public final class Constants {

    public final static String teamColor = "red";

    public final static class Limelight {
        public final static double LIMELIGHT_AIM_TOLERANCE = 10;
    }

    // Hood angle: 90-16.1 degrees
    public final static class Turret {
        public final static int PORT = 3;
        public final static double kP_CENTER = 0.21;

    }

    public final static class Drivetrain {
        // average encoder ticks per foot traveled.
        public static final double TICKS_PER_FOOT = 16200;

        public final static int RIGHT_MASTER_PORT = 5;
        public final static int LEFT_MASTER_PORT = 4;
        public final static int RIGHT_FOLLOWER_PORT = 7;
        public final static int LEFT_FOLLOWER_PORT = 6;
    }

    public final static class Launcher {
        public final static int MASTER_PORT = 1;
        public final static int FOLLOWER_PORT = 2;
        public final static int THRESHOLD = 1000;

        public final static double kGearRatio = 1.0 / 1.0;
        public final static double kWheelDiameterMeters = 0.1016;

        public final static double kP = 0.0001;
        public final static double kI = 0.0;
        public final static double kD = 0.0;

        public final static double kS = 0;
        public final static double kV = 0;
        public final static double kA = 0;
    }

    public final static class Intake {
        // solenoid ports on pcm
        public final static int FRONT_LEFT_SOLENOID_PORT = 6;
        public final static int FRONT_RIGHT_SOLENOID_PORT = 0;
        public final static int FRONT_INTAKE_PORT = 11;
    }

    public final static class Kicker {
        public final static int PORT = 8;
    }

    public final static class Chimney {
        public final static int PORT = 9;
    }

    public final static class Climber {
        public final static int LEFT_PORT = 13;
        public final static int RIGHT_PORT = 12;

        public final static int LEFT_ENCODER = 1;
        public final static int RIGHT_ENCODER = 2;
    }
}