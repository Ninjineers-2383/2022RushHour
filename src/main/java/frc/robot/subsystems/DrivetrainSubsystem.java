package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class DrivetrainSubsystem extends SubsystemBase {

    private final WPI_TalonFX rightMasterMotor = new WPI_TalonFX(Drivetrain.RIGHT_MASTER_PORT);
    private final WPI_TalonFX rightFollowerMotor = new WPI_TalonFX(Drivetrain.RIGHT_FOLLOWER_PORT);
    private final WPI_TalonFX leftMasterMotor = new WPI_TalonFX(Drivetrain.LEFT_MASTER_PORT);
    private final WPI_TalonFX leftFollowerMotor = new WPI_TalonFX(Drivetrain.LEFT_FOLLOWER_PORT);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
            leftMasterMotor,
            leftFollowerMotor);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
            leftMasterMotor,
            leftFollowerMotor);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    private boolean tippingProtectionEnabled = true;

    SlewRateLimiter throttleF = new SlewRateLimiter(3);
    SlewRateLimiter turnF = new SlewRateLimiter(2);

    private final DifferentialDriveOdometry odometry;

    // The gyro sensor
    private final AHRS rawGyro = new AHRS(SPI.Port.kMXP);
    public final Gyro m_gyro = rawGyro;

    public DrivetrainSubsystem() {
        rightMasterMotor.configFactoryDefault();
        rightFollowerMotor.configFactoryDefault();
        leftMasterMotor.configFactoryDefault();
        leftFollowerMotor.configFactoryDefault();

        rightMasterMotor.setSelectedSensorPosition(0);
        rightFollowerMotor.setSelectedSensorPosition(0);
        leftMasterMotor.setSelectedSensorPosition(0);
        leftFollowerMotor.setSelectedSensorPosition(0);

        rightMotors.setInverted(true);
        leftMotors.setInverted(false);

        rightMasterMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);
        leftMasterMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        rightFollowerMotor.follow(rightMasterMotor);
        leftFollowerMotor.follow(leftMasterMotor);

        odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    public void driveFeed() {
        drive.feed();
    }

    public void switchBrakeCoast(boolean isBrake) {
        NeutralMode mode = isBrake ? NeutralMode.Brake : NeutralMode.Coast;
        leftFollowerMotor.setNeutralMode(mode);
        leftMasterMotor.setNeutralMode(mode);
        rightMasterMotor.setNeutralMode(mode);
        rightFollowerMotor.setNeutralMode(mode);
    }

    public void periodic() {
        odometry.update(
                m_gyro.getRotation2d(), -encoderTicksToMeters(leftMasterMotor.getSelectedSensorPosition()),
                encoderTicksToMeters(rightMasterMotor.getSelectedSensorPosition()));

        SmartDashboard.putString("Gyro", m_gyro.getRotation2d().toString());
        SmartDashboard.putNumber("Left Encoder Meters",
                -encoderTicksToMeters(leftMasterMotor.getSelectedSensorPosition()));
        SmartDashboard.putNumber("Right Encoder Meters",
                encoderTicksToMeters(rightMasterMotor.getSelectedSensorPosition()));

        SmartDashboard.putString("Gyro", m_gyro.getRotation2d().toString());
        SmartDashboard.putNumber("Gyro pitch", rawGyro.getPitch());
        SmartDashboard.putNumber("Gyro yaw", rawGyro.getYaw());
        SmartDashboard.putNumber("Gyro roll", rawGyro.getRoll());
        SmartDashboard.putBoolean("tippingProtectionEnabled", tippingProtectionEnabled);

        SmartDashboard.putNumber("Velocity", getAverageVelocity().getAsDouble());
        SmartDashboard.putBoolean("Stopped", Math.abs(getAverageVelocity().getAsDouble()) < 0.1);

        SmartDashboard.putNumber("Left Master Sensor Pos", leftMasterMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Follower Senser Pos", leftFollowerMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Master Sensor Pos", rightMasterMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Follower Sensor Pos", rightFollowerMotor.getSelectedSensorPosition());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                -encoderTicksToMeters(leftMasterMotor.getSelectedSensorVelocity()),
                encoderTicksToMeters(rightMasterMotor.getSelectedSensorVelocity()));
    }

    public double encoderTicksToMeters(double ticks) {
        return ticks / 8.33 / 2048 * 0.3204;
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    public void resetEncoders() {
        leftMasterMotor.setSelectedSensorPosition(0);
        rightMasterMotor.setSelectedSensorPosition(0);
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public double getAverageEncoderDistance() {
        return (-encoderTicksToMeters(leftMasterMotor.getSelectedSensorPosition()) +
                encoderTicksToMeters(rightMasterMotor.getSelectedSensorPosition())) / 2.0;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public double getPitch() {
        return rawGyro.getPitch();
    }

    public double getRoll() {
        return rawGyro.getRoll();
    }

    // Arcade drive method. Forward and backward on left joystick and turn on right
    // joystick.
    public void drive(double power, double turn) {

        double driveOutput = throttleF.calculate(power);
        double kP = 0.02;
        double turnOutput = turnF.calculate(turn * 0.9);
        double roll = getRoll() * (Math.abs(getRoll()) > 2 ? kP : 0);

        if (tippingProtectionEnabled) {
            drive.arcadeDrive(driveOutput - roll, turnOutput, false);
        } else {
            drive.arcadeDrive(driveOutput, turnOutput, false);
        }
        // drive.arcadeDrive(power, 0.9 * turn);
    }

    public void toggleTippingEnabled() {
        tippingProtectionEnabled = true;
    }

    public void toggleTippingDisabled() {
        tippingProtectionEnabled = false;
    }

    public void driveSquared(double power, double turn) {

        double driveOutput = throttleF.calculate(power);
        double turnOutput = turnF.calculate(turn * 0.9);

        drive.arcadeDrive(driveOutput, turnOutput);
        // drive.arcadeDrive(power, 0.9 * turn);
    }

    // reset encoder positions to 0,
    public void zeroEncoders() {
        leftMasterMotor.setSelectedSensorPosition(0);
        rightMasterMotor.setSelectedSensorPosition(0);
    }

    public double getLeftPosition() {
        SmartDashboard.putNumber("left pos", leftMasterMotor.getSelectedSensorPosition());
        return leftMasterMotor.getSelectedSensorPosition();
    }

    public double getRightPosition() {
        SmartDashboard.putNumber("right pos", rightMasterMotor.getSelectedSensorPosition());
        return rightMasterMotor.getSelectedSensorPosition();
    }

    public DoubleSupplier getAverageVelocity() {
        return () -> (rightMasterMotor.getSelectedSensorVelocity(0) + leftMasterMotor.getSelectedSensorVelocity(0)) / 2;
    }

    public class SlewRateLimiter {
        private final double m_rateLimit;
        private double m_prevVal;
        private double m_prevTime;

        public SlewRateLimiter(double rateLimit, double initialValue) {
            m_rateLimit = rateLimit;
            m_prevVal = initialValue;
            m_prevTime = WPIUtilJNI.now() * 1e-6;
        }

        public SlewRateLimiter(double rateLimit) {
            this(rateLimit, 0);
        }

        public double calculate(double input) {
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            if (input > m_prevVal && m_prevVal > 0 || input < m_prevVal && m_prevVal < 0) {
                m_prevVal += MathUtil.clamp(input - m_prevVal, 1.3 * -m_rateLimit * elapsedTime,
                        1.3 * m_rateLimit * elapsedTime);
            }
            m_prevVal += MathUtil.clamp(input - m_prevVal, -m_rateLimit * elapsedTime, m_rateLimit * elapsedTime);
            m_prevTime = currentTime;
            return m_prevVal;
        }

        public void reset(double value) {
            m_prevVal = value;
            m_prevTime = WPIUtilJNI.now() * 1e-6;
        }
    }
}
