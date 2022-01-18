// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LauncherSubsystem extends SubsystemBase {

    WPI_TalonFX launcherMaster;
    WPI_TalonFX launcherFollower;

    /** Creates a new ExampleSubsystem. */
    public LauncherSubsystem() {
      launcherMaster = new WPI_TalonFX(RobotMap.LAUNCHER_MASTER_PORT);
      launcherFollower = new WPI_TalonFX(RobotMap.LAUNCHER_FOLLOWER_PORT);
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

    public void spin(double velocity) {
        launcherMaster.set(ControlMode.PercentOutput, velocity);
    }
}
