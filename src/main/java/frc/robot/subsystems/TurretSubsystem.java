package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TurretSubsystem extends SubsystemBase{

    WPI_TalonFX turret; 

    public TurretSubsystem() {
        turret = new WPI_TalonFX(RobotMap.TURRET_PORT);
    }

    public void turn(Double power) {
        turret.set(ControlMode.PercentOutput, power);
    }
}
