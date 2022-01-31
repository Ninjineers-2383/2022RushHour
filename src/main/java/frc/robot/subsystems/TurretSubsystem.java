package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Turret;
import frc.robot.commands.TurretCommand;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class TurretSubsystem extends SubsystemBase{
    private TalonSRX turret = new TalonSRX(RobotMap.TURRET_PORT);
    private boolean side = false;


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Turret pos", getCurrentPosition());
    }

    public TurretSubsystem() {
        turret.setInverted(false);
        turret.setSelectedSensorPosition(0);
    }


    public void setPower(Double power) {
        if (getCurrentPosition() > Turret.TURRET_INBOUNDS) {
            setPower(0.2);
            side = true;
        } else if (getCurrentPosition() < - Turret.TURRET_INBOUNDS) {
            setPower(-0.2);
            side = false;
        } else {
            turret.set(ControlMode.PercentOutput, power);
        }
        
        SmartDashboard.putBoolean("Side", side);
    }
    

    public void setPower(Double power, double delay) {
        turret.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Turret Power", power);
        Timer.delay(delay);
    }


    public void seek() {
        setPower(side ? 1:-1 * Turret.SEEKING_POWER);
    }

    
    public double getCurrentPosition() {
        return turret.getSelectedSensorPosition(0);
    }
}
