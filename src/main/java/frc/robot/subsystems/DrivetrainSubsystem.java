package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.helpers.SlewRateLimiter;

public class DrivetrainSubsystem extends SubsystemBase {

    // creates four motor instances with a Talon FX motor controller
    private final WPI_TalonFX rightMasterMotor;
    private final WPI_TalonFX rightFollowerMotor;
    private final WPI_TalonFX leftMasterMotor;
    private final WPI_TalonFX leftFollowerMotor;

    // creates two motor groups
    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;

    // creates an instance of differential drive using the two motor groups
    private final DifferentialDrive drive;

    // boolean that tells if the slew rates are enabled or not
    private boolean tippingProtectionEnabled;

    // creates the slew rate instances for forward and turn
    private final SlewRateLimiter throttleF;
    private final SlewRateLimiter turnF;

    // creates an instance of differential drive odometry
    private final DifferentialDriveOdometry odometry;

    // The gyro sensor
    private final AHRS rawGyro;
    public final Gyro m_gyro;

    /**
     * Constructor for the drivetrain subsystem
     */
    public DrivetrainSubsystem() {
        rightMasterMotor = new WPI_TalonFX(Drivetrain.RIGHT_MASTER_PORT);
        rightFollowerMotor = new WPI_TalonFX(Drivetrain.RIGHT_FOLLOWER_PORT);
        leftMasterMotor = new WPI_TalonFX(Drivetrain.LEFT_MASTER_PORT);
        leftFollowerMotor = new WPI_TalonFX(Drivetrain.LEFT_FOLLOWER_PORT);

        leftMotors = new MotorControllerGroup(
                leftMasterMotor,
                leftFollowerMotor);
        rightMotors = new MotorControllerGroup(
                rightMasterMotor,
                rightFollowerMotor);

        drive = new DifferentialDrive(leftMotors, rightMotors);

        tippingProtectionEnabled = true;

        throttleF = new SlewRateLimiter(3);
        turnF = new SlewRateLimiter(2);

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

        rawGyro = new AHRS(SPI.Port.kMXP);
        m_gyro = rawGyro;

        odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        odometry.update(
                m_gyro.getRotation2d(), encoderTicksToMeters(leftMasterMotor.getSelectedSensorPosition()),
                -encoderTicksToMeters(rightMasterMotor.getSelectedSensorPosition()));

        SmartDashboard.putString("Gyro", m_gyro.getRotation2d().toString());
        SmartDashboard.putNumber("Left Encoder Meters",
                encoderTicksToMeters(leftMasterMotor.getSelectedSensorPosition()));
        SmartDashboard.putNumber("Right Encoder Meters",
                -encoderTicksToMeters(rightMasterMotor.getSelectedSensorPosition()));

        SmartDashboard.putString("Gyro", m_gyro.getRotation2d().toString());
        SmartDashboard.putNumber("Gyro pitch", rawGyro.getPitch());
        SmartDashboard.putNumber("Gyro yaw", rawGyro.getYaw());
        SmartDashboard.putNumber("Gyro roll", rawGyro.getRoll());
        SmartDashboard.putBoolean("tippingProtectionEnabled", tippingProtectionEnabled);

        SmartDashboard.putNumber("Velocity", getAverageVelocity().getAsDouble());
        SmartDashboard.putBoolean("Stopped", Math.abs(getAverageVelocity().getAsDouble()) < 0.1);

        SmartDashboard.putNumber("Left Master Sensor Pos", leftMasterMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Follower Sensor Pos", leftFollowerMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Master Sensor Pos", rightMasterMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Follower Sensor Pos", rightFollowerMotor.getSelectedSensorPosition());
    }

    /**
     * 
     * @param isBrake toggles the drivetrain between coast and brake
     */
    public void switchBrakeCoast(boolean isBrake) {
        NeutralMode mode = isBrake ? NeutralMode.Brake : NeutralMode.Coast;
        leftFollowerMotor.setNeutralMode(mode);
        leftMasterMotor.setNeutralMode(mode);
        rightMasterMotor.setNeutralMode(mode);
        rightFollowerMotor.setNeutralMode(mode);
    }

    /**
     * 
     * @return the odometry pose in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * 
     * @return the wheel speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                encoderTicksToMeters(leftMasterMotor.getSelectedSensorVelocity()),
                -encoderTicksToMeters(rightMasterMotor.getSelectedSensorVelocity()));
    }

    /**
     * 
     * @param ticks raw encoder ticks
     * @return meters based on the gear ratios of the drivetrain
     */
    public double encoderTicksToMeters(double ticks) {
        return ticks / 8.33 / 2048 * 0.3204;
    }

    /**
     * Resets the odometry to a given pose
     * 
     * @param pose the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Resets the encoders of the robot
     */
    public void resetEncoders() {
        SmartDashboard.putNumber("Right Master Reset", rightMasterMotor.setSelectedSensorPosition(0).value);
        SmartDashboard.putNumber("Right Follower Reset", rightFollowerMotor.setSelectedSensorPosition(0).value);
        SmartDashboard.putNumber("Left Master Reset", leftMasterMotor.setSelectedSensorPosition(0).value);
        SmartDashboard.putNumber("Left Follower Reset", leftFollowerMotor.setSelectedSensorPosition(0).value);
    }

    /**
     * Drives the robot in tank drive fashion
     * 
     * @param left  runs the left motors at this speed
     * @param right runs the right motors at this speed
     */
    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    /**
     * Drives the robot in tank drive fashion but uses motor voltages as inputs
     * 
     * @param leftVolts  the volts of the left side
     * @param rightVolts the volts of the right side
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /**
     * Gets the average of the encoder ticks in meters
     * 
     * @return the average encoder distance
     */
    public double getAverageEncoderDistance() {
        return (encoderTicksToMeters(leftMasterMotor.getSelectedSensorPosition()) +
                -encoderTicksToMeters(rightMasterMotor.getSelectedSensorPosition())) / 2.0;
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

    /**
     * Zeroes the heading of the robot.
     */
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

    /**
     * Gets the pitch of the gyro
     * 
     * @return the pitch of the gyro
     */
    public double getPitch() {
        return rawGyro.getPitch();
    }

    /**
     * Gets the roll of the gyro
     * 
     * @return the roll of the gyro
     */
    public double getRoll() {
        return rawGyro.getRoll();
    }

    /**
     * Drives the robot using differential drive controls
     * 
     * @param throttle the forward component of the drive
     * @param turn     the turn component of the drive
     */
    public void drive(double throttle, double turn) {

        double driveOutput = throttleF.calculate(throttle);
        double kP = 0.02;
        double turnOutput = turnF.calculate(turn * 0.9);
        double roll = getRoll() * (Math.abs(getRoll()) > 2 ? kP : 0);

        if (tippingProtectionEnabled) {
            drive.arcadeDrive(driveOutput - roll, turnOutput, false);
        } else {
            drive.arcadeDrive(driveOutput, turnOutput, false);
        }
    }

    /**
     * Enables tipping protection
     */
    public void toggleTippingEnabled() {
        tippingProtectionEnabled = true;
    }

    /**
     * Disables tipping protection
     */
    public void toggleTippingDisabled() {
        tippingProtectionEnabled = false;
    }

    /**
     * Gets the position of the left drivetrain
     */
    public double getLeftPosition() {
        SmartDashboard.putNumber("left pos", leftMasterMotor.getSelectedSensorPosition());
        return leftMasterMotor.getSelectedSensorPosition();
    }

    /**
     * Gets the position of the right drivetrain
     */
    public double getRightPosition() {
        SmartDashboard.putNumber("right pos", rightMasterMotor.getSelectedSensorPosition());
        return rightMasterMotor.getSelectedSensorPosition();
    }

    /**
     * Gets the average velocity of both sides of the drivetrain
     * 
     * @return the average velocity of the right and left side of the drivetrain
     */
    public DoubleSupplier getAverageVelocity() {
        return () -> (rightMasterMotor.getSelectedSensorVelocity(0) + leftMasterMotor.getSelectedSensorVelocity(0)) / 2;
    }
}
