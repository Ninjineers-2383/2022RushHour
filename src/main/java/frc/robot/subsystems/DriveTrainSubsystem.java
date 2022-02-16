package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DrivetrainSubsystem extends SubsystemBase {
  public final WPI_TalonFX rightMasterMotor    = new WPI_TalonFX(Drivetrain.RIGHT_MASTER_PORT);
  public final WPI_TalonFX rightFollowerMotor  = new WPI_TalonFX(Drivetrain.RIGHT_FOLLOWER_PORT);
  public final WPI_TalonFX leftMasterMotor     = new WPI_TalonFX(Drivetrain.LEFT_MASTER_PORT);
  public final WPI_TalonFX leftFollowerMotor   = new WPI_TalonFX(Drivetrain.LEFT_FOLLOWER_PORT);

  SlewRateLimiter throttleF = new SlewRateLimiter(1);
  SlewRateLimiter turnF = new SlewRateLimiter(2);
  
  private DifferentialDrive drive;

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(Drivetrain.LEFT_MASTER_PORT,
      Drivetrain.LEFT_FOLLOWER_PORT);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(Drivetrain.RIGHT_MASTER_PORT,
      Drivetrain.RIGHT_FOLLOWER_PORT);
  
    // The gyro sensor
    private final Gyro m_gyro = new AHRS();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

  
  public DrivetrainSubsystem() {
    rightMasterMotor  .configFactoryDefault();
    rightFollowerMotor.configFactoryDefault();
    leftMasterMotor   .configFactoryDefault();
    leftFollowerMotor .configFactoryDefault();

    rightMasterMotor  .setInverted(true);
    rightFollowerMotor.setInverted(true);
    leftMasterMotor   .setInverted(false);
    leftFollowerMotor .setInverted(false);

    // //Configure P values
    // rightMasterMotor  .config_kP(0, Drivetrain.Motor_kP);
    // rightFollowerMotor.config_kP(0, Drivetrain.Motor_kP);
    // leftMasterMotor   .config_kP(0, Drivetrain.Motor_kP);
    // leftFollowerMotor .config_kP(0, Drivetrain.Motor_kP);

    // //Configure I values
    // rightMasterMotor  .config_kI(0, Drivetrain.Motor_kI);
    // rightFollowerMotor.config_kI(0, Drivetrain.Motor_kI);
    // leftMasterMotor   .config_kI(0, Drivetrain.Motor_kI);
    // leftFollowerMotor .config_kI(0, Drivetrain.Motor_kI);

    // //Configure D values
    // rightMasterMotor  .config_kD(0, Drivetrain.Motor_kD);
    // rightFollowerMotor.config_kD(0, Drivetrain.Motor_kD);
    // leftMasterMotor   .config_kD(0, Drivetrain.Motor_kD);
    // leftFollowerMotor .config_kD(0, Drivetrain.Motor_kD);

    rightMasterMotor  .setNeutralMode(NeutralMode.Brake);
    rightFollowerMotor.setNeutralMode(NeutralMode.Brake);
    leftMasterMotor   .setNeutralMode(NeutralMode.Brake);
    leftFollowerMotor .setNeutralMode(NeutralMode.Brake);

    rightFollowerMotor.follow(rightMasterMotor);
    leftFollowerMotor .follow(leftMasterMotor);

    drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.Drivetrain.ENCODER_DISTANCE_PER_PULSE);
    m_rightEncoder.setDistancePerPulse(Constants.Drivetrain.ENCODER_DISTANCE_PER_PULSE);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    SmartDashboard.putNumber("Left Master Sensor Pos", leftMasterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Follower Senser Pos", leftFollowerMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Master Sensor Pos", rightMasterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Follower Sensor Pos", rightFollowerMotor.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  //reset odometry to a pose passed as an argument
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void VelocityOutput(int leftVelocity, int rightVelocity) {
    rightMasterMotor.set(ControlMode.Velocity, rightVelocity);
    rightFollowerMotor.set(ControlMode.Velocity, rightVelocity);
    leftMasterMotor.set(ControlMode.Velocity, leftVelocity);
    leftFollowerMotor.set(ControlMode.Velocity, leftVelocity);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMasterMotor.setVoltage(leftVolts);
    rightMasterMotor.setVoltage(rightVolts);
    leftFollowerMotor.setVoltage(leftVolts);
    rightFollowerMotor.setVoltage(rightVolts);
    drive.feed();

    SmartDashboard.putNumber("LM Volts", leftMasterMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("RM Volts", rightMasterMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("LF Volts", leftFollowerMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("RF Volts", rightFollowerMotor.getMotorOutputVoltage());
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  //defining the maximum output
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  // Arcade drive method. Forward and backward on left joystick and turn on right joystick.
  public void drive(double power, double turn){
    drive.arcadeDrive(throttleF.calculate(power), turnF.calculate(turn * 0.9));
  }

  public void drive(int left, int right, int power, int time) {
    rightMasterMotor.set(ControlMode.Position, right);
    rightFollowerMotor.set(ControlMode.Position, right);
    leftMasterMotor.set(ControlMode.Position, left);
    leftFollowerMotor.set(ControlMode.Position, left);

    rightMasterMotor.set(power);
    leftMasterMotor.set(power);

    Timer.delay(time);

    rightMasterMotor.set(0);
    rightFollowerMotor.set(0);
    leftMasterMotor.set(0);
    leftFollowerMotor.set(0);
  }

  // reset encoder positions to 0
  public void zeroEncoders() {
		leftMasterMotor.setSelectedSensorPosition(0);
		rightMasterMotor.setSelectedSensorPosition(0);
  }
  
  
  public double getLeftPosition() {
		return leftMasterMotor.getSelectedSensorPosition();
	}

	public double getRightPosition() {
		return rightMasterMotor.getSelectedSensorPosition();
  }
}