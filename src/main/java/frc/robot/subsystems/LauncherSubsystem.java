package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Launcher;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class LauncherSubsystem extends SubsystemBase {
    private final WPI_TalonFX masterMotor = new WPI_TalonFX(Launcher.MASTER_PORT);
    private final WPI_TalonFX followerMotor  = new WPI_TalonFX(Launcher.FOLLOWER_PORT);

    public LauncherSubsystem() {
      masterMotor.setNeutralMode(NeutralMode.Coast);
      followerMotor.setNeutralMode(NeutralMode.Coast);

      masterMotor.setInverted(false);
      followerMotor.setInverted(false);
      
      followerMotor.follow(masterMotor);
    }


    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter Velocity", masterMotor.getSelectedSensorVelocity());
    }


    // spin up flywheels
    public void spin(double velocity) {
      if (velocity == 0) {
        masterMotor.set(ControlMode.PercentOutput, 0);
      } else {
        masterMotor.set(ControlMode.Velocity, velocity);
      }
    }

    public boolean isReady(){
      return masterMotor.getSelectedSensorVelocity() >= masterMotor.getClosedLoopTarget() - 100 
      && masterMotor.getSelectedSensorVelocity() <= masterMotor.getClosedLoopTarget() + 100
      && followerMotor.getSelectedSensorVelocity() >= followerMotor.getClosedLoopTarget() - 100
      && followerMotor.getSelectedSensorVelocity() <= followerMotor.getClosedLoopTarget() + 100;
    }
}
