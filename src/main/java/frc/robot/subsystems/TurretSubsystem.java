package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.commands.TurretCommand;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TurretSubsystem extends SubsystemBase{

    public TalonSRX turret;

    public TurretSubsystem() {
        turret = new TalonSRX(RobotMap.TURRET_PORT);
        turret.setInverted(false);
    }

    public void turn(Double power) {
        turret.set(ControlMode.PercentOutput, power);
    }

    public void forceTurn(Double power, double delay) {
        turret.set(ControlMode.PercentOutput, power);
        Timer.delay(delay);
    }

    public void periodic(){
        SmartDashboard.putNumber("Turret pos", getCurrentPosition());
        SmartDashboard.putNumber("Power", TurretCommand.range);
    }
        
    public double getCurrentPosition() {
        return turret.getSelectedSensorPosition(0);
    }
}
