package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;

public class KickerSubsystem extends SubsystemBase {
    // create motor instance that uses a TalonSRX motor controller
    private final VictorSPX kicker = new VictorSPX(Kicker.PORT);

    /**
     * Kicker subsystem constructor
     */
    public KickerSubsystem() {
        kicker.setInverted(false);
        kicker.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("kicker Velocity", kicker.getSelectedSensorVelocity());
    }

    /**
     * Sets the power of the kicker from -1 to 1
     * 
     * @param power power of the kicker
     */
    public void setPower(Double power) {
        kicker.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("kicker power", power);
    }
}