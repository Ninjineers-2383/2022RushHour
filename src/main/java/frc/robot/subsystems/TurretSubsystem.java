package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;

public class TurretSubsystem extends SubsystemBase {
    // changes states based on whether or not the turret is over bounds or under
    // bounds
    public enum TurretBoundsState {
        OverBounds,
        UnderBounds
    }

    // creates a motor instance using a TalonSRX motor controller
    private TalonSRX motor = new TalonSRX(Turret.PORT);

    // the state of the bounds
    private TurretBoundsState boundsState = TurretBoundsState.OverBounds;

    /**
     * Turret subsystem constructor
     */
    public TurretSubsystem() {
        motor.setInverted(false);
        motor.setSensorPhase(true);
        motor.setSelectedSensorPosition(0);
        brake();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret pos", getCurrentPosition());
    }

    /**
     * Sets the turret to coast
     */
    public void coast() {
        motor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets the turret to brake
     */
    public void brake() {
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the power of the turret
     * 
     * @param power power of the turret in velocity
     */
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

    /**
     * Rotates the turret until it hits the bound, and rotates the other way until
     * it hits that bound
     */
    public void seek() {
        setPower(boundsState == TurretBoundsState.OverBounds ? Turret.SEEKING_POWER : -Turret.SEEKING_POWER);
    }

    /**
     * Sets the direction of the turret
     * 
     * @param direction direction of the turret
     */
    public void seekDirection(boolean direction) {
        if (direction) {
            boundsState = TurretBoundsState.OverBounds;
        } else {
            boundsState = TurretBoundsState.UnderBounds;
        }
    }

    /**
     * Runs the turret to a specific position
     * 
     * @param position position in encoder ticks
     * @return whether or not the turret is at the position
     */
    public boolean runToPosition(int position) {
        double error = -this.getCurrentPosition() - position;
        if (Math.abs(error) > 100) {
            setPower(Turret.kP_CENTER * error);
            return false;
        } else {
            setPower(0.0);
            return true;
        }
    }

    /**
     * Gets the current position of the turret
     * 
     * @return current position of the turret
     */
    public double getCurrentPosition() {
        return motor.getSelectedSensorPosition(0);
    }
}
