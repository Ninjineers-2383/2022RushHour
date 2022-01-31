package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.Drivetrain;


public class DrivetrainSubsystem extends SubsystemBase {
  private final WPI_TalonFX rightMasterMotor    = new WPI_TalonFX(Drivetrain.RIGHT_MASTER_PORT);
  private final WPI_TalonFX rightFollowerMotor  = new WPI_TalonFX(Drivetrain.RIGHT_FOLLOWER_PORT);
  private final WPI_TalonFX leftMasterMotor     = new WPI_TalonFX(Drivetrain.LEFT_MASTER_PORT);
  private final WPI_TalonFX leftFollowerMotor   = new WPI_TalonFX(Drivetrain.LEFT_FOLLOWER_PORT);
  
  private DifferentialDrive drive;

  
  public DrivetrainSubsystem() {
    rightMasterMotor  .setInverted(true);
    rightFollowerMotor.setInverted(true);
    leftMasterMotor   .setInverted(false);
    leftFollowerMotor .setInverted(false);

    rightMasterMotor  .setNeutralMode(NeutralMode.Brake);
    rightFollowerMotor.setNeutralMode(NeutralMode.Brake);
    leftMasterMotor   .setNeutralMode(NeutralMode.Brake);
    leftFollowerMotor .setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
  }

  // Arcade drive method. Forward and backward on left joystick and turn on right joystick.
  public void drive(double power, double turn){
    drive.arcadeDrive(power, turn * 0.9);
  }
  // autonomous driving
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
