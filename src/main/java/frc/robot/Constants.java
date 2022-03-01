package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


public final class Constants {

    public final static class Limelight {
        public final static double LIMELIGHT_AIM_TOLERANCE = 10;
    }

    //Hood angle: 90-16.1 degrees
    public final static class Turret {
        public final static int PORT = 3;
        public final static int FULL_ROTATION = 32000;
        public final static int BOUNDS = FULL_ROTATION + 10000;
        public final static double SEEKING_POWER = 0.7;
        public final static double kP = 0.02;
        public final static double DRIVE_VELOCITY_FACTOR = 0;
        public final static double OFFSET_TICKS = -1000;
    } 


    public final static class Drivetrain {
        //average encoder ticks per foot travelled.
        public static final double TICKS_PER_FOOT = 16200;
        // PID thing
        public static final double Ramset_kP = 0;
        // PID Values
        // Before Big John touched this P=.01, I=.001, D=0
        public static final double Motor_kP = 0.069715;
        public static final double Motor_kI = 0;
        public static final double Motor_kD = 0;

        public static final double kTrackwidthMeters = 0.62865;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        // 0.683 volts right side ks
        // 0.780 volts left side ks
        // 0.83 volts right side kv
        // 0.87 volts left side kv

        // Before Big John touched this kV = .66, kVspm = .35, kvsspm = .1
        public static final double ksVolts = 2; //.78
        public static final double ksPercent = 0.26;
        public static final double ksPercentTurn = 0.35;
        public static final double kvVoltSecondsPerMeter = 0.072357; //.95s
        public static final double kaVoltSecondsSquaredPerMeter = 0.006527; //0.208

        public final static double DRIVETRAIN_GEAR_RATIO = 8.33;
        public final static double WHEEL_DIAMETER_METERS = 0.1016;
        public final static int ENCODER_CPR = 2048;
        public final static double ENCODER_DISTANCE_PER_PULSE = 
        (((WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR)) / DRIVETRAIN_GEAR_RATIO;

        public final static int RIGHT_MASTER_PORT = 4;
        public final static int LEFT_MASTER_PORT = 5;
        public final static int RIGHT_FOLLOWER_PORT = 6;
        public final static int LEFT_FOLLOWER_PORT = 7;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }

    public final static class Launcher {
        public final static int MASTER_PORT = 1;
        public final static int FOLLOWER_PORT = 2;
    }

    public final static class Intake {
        //solenoid ports on pcm
        public final static int FRONT_LEFT_SOLENOID_PORT = 1;
        public final static int REAR_LEFT_SOLENOID_PORT = 6;
        public final static int FRONT_RIGHT_SOLENOID_PORT = 0;
        public final static int REAR_RIGHT_SOLENOID_PORT = 7;
        public final static int FRONT_INTAKE_PORT = 10;
        public final static int REAR_INTAKE_PORT = 11;
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
        public final static int HOOK_PORT = 14;

        public final static int LEFT_ENCODER = 1;
        public final static int RIGHT_ENCODER = 2;
    }
}