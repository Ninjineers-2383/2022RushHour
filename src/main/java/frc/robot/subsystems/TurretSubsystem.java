package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.Turret;


public class TurretSubsystem extends SubsystemBase{
    private TalonSRX motor = new TalonSRX(Turret.PORT);
    private boolean side = false;


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Turret pos", getCurrentPosition());
    }

    public TurretSubsystem() {
        motor.setInverted(false);
        motor.setSelectedSensorPosition(0);
    }


    public void setPower(Double power) {
        SmartDashboard.putNumber("446pm", power);
        if (getCurrentPosition() > Turret.BOUNDS) {
            power = 0.2;
            this.side = true;
        } else if (getCurrentPosition() <= 0) {
            power = -0.2;
            this.side = false;
        }
        motor.set(ControlMode.PercentOutput, power);
        
        SmartDashboard.putBoolean("Side", side);
    }

    // Rotates til side flips, then rotates other direction
    public void seek() {
        setPower(this.side ? 1:-1 * Turret.SEEKING_POWER);
    }

    
    public double getCurrentPosition() {
        return motor.getSelectedSensorPosition(0);
    }
}