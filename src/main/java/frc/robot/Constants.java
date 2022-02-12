package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


public final class Constants {

    public final static class Limelight {
        public final static double LIMELIGHT_AIM_TOLERANCE = 18;
    }

    //Hood angle: 90-16.1 degrees
    public final static class Turret {
        public final static int PORT = 3;
        public final static int BOUNDS = 65100;
        public final static double SEEKING_POWER = 0.4;
    } 


    public final static class Drivetrain {
        public static final double kTrackwidthMeters = 0.62865;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        // 0.683 volts right side ks
        // 0.780 volts left side ks
        // 0.83 volts right side kv
        // 0.87 volts left side kv

        public static final double ksVolts = 0.78;
        public static final double kvVoltSecondsPerMeter = 0.95;
        public static final double kaVoltSecondsSquaredPerMeter = 0.208;
        public static final double kPDriveVel = 8.5;

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
        public static final double kRamseteB = 1.45;
        public static final double kRamseteZeta = 0.45;
      }

    public final static class Launcher {
        public final static int MASTER_PORT = 1;
        public final static int FOLLOWER_PORT = 2;
    }

    public final static class Intake {
        //solenoid ports on pcm
        public final static int FRONT_LEFT_SOLENOID_PORT = 7;
        public final static int REAR_LEFT_SOLENOID_PORT = 4;
        public final static int FRONT_RIGHT_SOLENOID_PORT = 6;
        public final static int REAR_RIGHT_SOLENOID_PORT = 5;
        public final static int FRONT_INTAKE_PORT = 10;
        public final static int REAR_INTAKE_PORT = 11;
    }


    public final static class Kicker {
        public final static int PORT = 8;
    }
    

    public final static class Chimney {
        public final static int PORT = 9;
    }
}