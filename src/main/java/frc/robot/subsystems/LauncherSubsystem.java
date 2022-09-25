package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Launcher;

public class LauncherSubsystem extends SubsystemBase {
    // creates two motor instances using a TalonFX motor controller
    private final WPI_TalonFX masterMotor = new WPI_TalonFX(Launcher.MASTER_PORT);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(Launcher.FOLLOWER_PORT);

    /**
     * Launcher subsystem constructor
     */
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
        SmartDashboard.putNumber("Shooter Target", masterMotor.getClosedLoopTarget());
    }

    /**
     * Spins the flywheels at a certain velocity
     * 
     * @param velocity the velocity of the launcher
     */
    public void spin(double velocity) {
        if (velocity == 0) {
            masterMotor.set(ControlMode.PercentOutput, 0);
        } else {
            masterMotor.set(ControlMode.Velocity, velocity);
        }
    }

    /**
     * Determines whether or not the launcher is ready to receive a ball from the
     * kicker
     * 
     * @return whether or not the launcher is ready
     */
    public boolean isReady() {
        boolean ready = false;
        if (masterMotor.getControlMode() == ControlMode.Velocity) {
            ready = masterMotor.getSelectedSensorVelocity() >= masterMotor.getClosedLoopTarget() - Launcher.THRESHOLD
                    && masterMotor.getSelectedSensorVelocity() <= masterMotor.getClosedLoopTarget()
                            + Launcher.THRESHOLD;
        }
        SmartDashboard.putBoolean("Launcher Is Ready", ready);
        return ready;
    }
}
