// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
    rightMasterMotor  .setInverted(false);
    leftMasterMotor   .setInverted(false);
    rightFollowerMotor.setInverted(false);
    leftFollowerMotor .setInverted(false);

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

  // Timed Auto Method
  // public void autoDrive(double left, double right, double duration) {
  //   rightMasterMotor.set(ControlMode.PercentOutput,   -right);
  //   rightFollowerMotor.set(ControlMode.PercentOutput, -right);
  //   leftMasterMotor.set(ControlMode.PercentOutput,     left);
  //   leftFollowerMotor.set(ControlMode.PercentOutput,   left);
  //   Timer.delay(duration); 
  //   rightMasterMotor.set(ControlMode.PercentOutput, 0);
  //   rightFollowerMotor.set(ControlMode.PercentOutput, 0);
  //   leftMasterMotor.set(ControlMode.PercentOutput, 0);
  //   leftFollowerMotor.set(ControlMode.PercentOutput, 0);
  // }
}
