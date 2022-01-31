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
        if (getCurrentPosition() > Turret.BOUNDS) {
            setPower(0.2);
            side = true;
        } else if (getCurrentPosition() < - Turret.BOUNDS) {
            setPower(-0.2);
            side = false;
        } else {
            motor.set(ControlMode.PercentOutput, power);
        }
        
        SmartDashboard.putBoolean("Side", side);
    }

    // Rotates til side flips, then rotates other direction
    public void seek() {
        setPower(side ? 1:-1 * Turret.SEEKING_POWER);
    }

    
    public double getCurrentPosition() {
        return motor.getSelectedSensorPosition(0);
    }
}
