package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;

public class TurretSubsystem extends SubsystemBase {
    public enum TurretBoundsState {
        OverBounds,
        UnderBounds
    }

    private TalonSRX motor = new TalonSRX(Turret.PORT);
    private TurretBoundsState boundsState = TurretBoundsState.OverBounds;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret pos", getCurrentPosition());
    }

    public TurretSubsystem() {
        motor.setInverted(false);
        motor.setSensorPhase(true);
        motor.setSelectedSensorPosition(0);
        brake();
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
            power = -500.0;
            boundsState = TurretBoundsState.UnderBounds;
        } else if (getCurrentPosition() < -Turret.BOUNDS) {
            power = 500.0;
            boundsState = TurretBoundsState.OverBounds;
        }
        motor.set(ControlMode.Velocity, power);
        SmartDashboard.putNumber("446pm", power);
        SmartDashboard.putString("Side", boundsState.toString());
    }

    // Rotates til side flips, then rotates other direction
    public void seek() {
        setPower(boundsState == TurretBoundsState.OverBounds ? Turret.SEEKING_POWER : -Turret.SEEKING_POWER);
    }

    public void seekDirection(boolean direction) {
        if (direction) {
            boundsState = TurretBoundsState.OverBounds;
        } else {
            boundsState = TurretBoundsState.UnderBounds;
        }
    }

    public void runToPosition(int position) {
        double error = -this.getCurrentPosition() - position;
        if (Math.abs(error) > 100) {
            setPower(Turret.kP_CENTER * error);
        } else {
            setPower(0.0);
        }
    }

    public double getCurrentPosition() {
        return motor.getSelectedSensorPosition(0);
    }
}
