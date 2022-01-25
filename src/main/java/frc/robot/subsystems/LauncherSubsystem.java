// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LauncherSubsystem extends SubsystemBase {

    // Creates the two motors that uses built-in TalonFX motor controllers
    WPI_TalonFX launcherMaster;
    WPI_TalonFX launcherFollower;

    // Launcher Subsystem is created. Set parameters like port value, mode, inversion, and follows are defined here.
    public LauncherSubsystem() {
      launcherMaster = new WPI_TalonFX(RobotMap.LAUNCHER_MASTER_PORT);
      launcherFollower = new WPI_TalonFX(RobotMap.LAUNCHER_FOLLOWER_PORT);
      launcherMaster.setNeutralMode(NeutralMode.Coast);
      launcherFollower.setNeutralMode(NeutralMode.Coast);

      launcherMaster.setInverted(true);
      launcherFollower.setInverted(true);
      
      launcherFollower.follow(launcherMaster);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Shooter Velocity", launcherMaster.getSelectedSensorVelocity());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    // void method that takes in a velocity and commands the launcher module to do something.
    public void spin(double velocity) {
      if (velocity == 0) {
        launcherMaster.set(ControlMode.PercentOutput, 0);
      } else {
        launcherMaster.set(ControlMode.Velocity, velocity);
      }
    }
}
