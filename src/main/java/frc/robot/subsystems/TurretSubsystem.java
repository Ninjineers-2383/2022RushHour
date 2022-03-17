package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;

public class TurretSubsystem extends SubsystemBase {
    private TalonSRX motor = new TalonSRX(Turret.PORT);
    private boolean side = false;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret pos", getCurrentPosition());
    }

    public TurretSubsystem() {
        motor.setInverted(false);
        motor.setSelectedSensorPosition(0);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void setPosition(int pos) {
        motor.setSelectedSensorPosition(pos);
    }

    public void coast() {
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setPower(Double power) {
        if (getCurrentPosition() > Turret.BOUNDS) {
            power = 0.3;
            this.side = true;
        } else if (getCurrentPosition() <= -Turret.BOUNDS) {
            power = -0.3;
            this.side = false;
        }
        motor.set(ControlMode.PercentOutput, power);

        SmartDashboard.putNumber("446pm", power);
        SmartDashboard.putBoolean("Side", side);
    }

    // Rotates til side flips, then rotates other direction
    public void seek(boolean flipSides) {
        if (flipSides) {
            setPower(this.side ? -1 * Turret.SEEKING_POWER : 1 * Turret.SEEKING_POWER);
        } else {
            setPower(this.side ? 1 * Turret.SEEKING_POWER : -1 * Turret.SEEKING_POWER);
        }
    }

    public void runToPosition(int position) {
        double error = this.getCurrentPosition() - position;
        if (Math.abs(error) > 100) {
            setPower(Turret.kP_CENTER * error + Turret.kS * Math.signum(error));
        } else {
            setPower(0.0);
        }
    }

    public double getCurrentPosition() {
        return motor.getSelectedSensorPosition(0);
    }
}
