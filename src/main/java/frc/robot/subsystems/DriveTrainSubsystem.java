// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.RobotMap;


public class DriveTrainSubsystem extends SubsystemBase {

  WPI_TalonFX rightMasterMotor    = new WPI_TalonFX(RobotMap.RIGHT_MASTER_DRIVE_PORT);
  WPI_TalonFX rightFollowerMotor    = new WPI_TalonFX(RobotMap.RIGHT_MASTER_DRIVE_PORT);
  WPI_TalonFX leftMasterMotor   = new WPI_TalonFX(RobotMap.LEFT_MASTER_DRIVE_PORT);
  WPI_TalonFX leftFollowerMotor   = new WPI_TalonFX(RobotMap.LEFT_MASTER_DRIVE_PORT);
  
  private DifferentialDrive drive;

  /** Creates a new ExampleSubsystem. */
  public DriveTrainSubsystem() {
    rightMasterMotor  .setInverted(true);
    leftMasterMotor   .setInverted(false);
    rightFollowerMotor.setInverted(true);
    leftFollowerMotor .setInverted(false);

    rightMasterMotor  .setNeutralMode(NeutralMode.Brake);
    rightFollowerMotor.setNeutralMode(NeutralMode.Brake);
    leftMasterMotor   .setNeutralMode(NeutralMode.Brake);
    leftFollowerMotor .setNeutralMode(NeutralMode.Brake);

    drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
  }

  public void arcade(double power, double turn){
    // Arcade drive method. Forward and backward on left joystick and turn on right joystick.
    drive.arcadeDrive(power, turn * 0.9);
  }

  @Override
  public void periodic() {
    // This method will be called once per schedsuler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetEncoders() {
		leftMasterMotor.setSelectedSensorPosition(0);
		rightMasterMotor.setSelectedSensorPosition(0);
  }

  public double getLeftPosition() {
		return leftMasterMotor.getSelectedSensorPosition();
	}
	
	public double getRightPosition() {
		return rightMasterMotor.getSelectedSensorPosition();
  }

  // Auto Method
  public void drive(int left, int right, int power, int time) {

    rightMasterMotor.set(ControlMode.Position, right);
    rightFollowerMotor.set(ControlMode.Position, right);
    leftMasterMotor.set(ControlMode.Position, left);
    leftFollowerMotor.set(ControlMode.Position, left);

    rightMasterMotor.set(power);
    rightFollowerMotor.set(power);
    leftMasterMotor.set(power);
    leftFollowerMotor.set(power);

    Timer.delay(time);

    rightMasterMotor.set(0);
    rightFollowerMotor.set(0);
    leftMasterMotor.set(0);
    leftFollowerMotor.set(0);
    
  }
}
