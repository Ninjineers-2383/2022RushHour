package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;

public class KickerSubsystem extends SubsystemBase {
    // create kicker instance that uses a TalonSRX motor controller.
    private final VictorSPX kicker = new VictorSPX(Kicker.PORT);

    // Kicker subsystem defined
    public KickerSubsystem() {
        kicker.setInverted(false);
        kicker.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("kicker Velocity", kicker.getSelectedSensorVelocity());
    }

    // method that returns nothing (void) but sets the kicker at a set power in the
    // interval of [-1, 1]
    public void setPower(Double power) {
        kicker.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("kicker power", power);
    }

    // method that returns nothing (void) but sets the kicker at a set velocity.
    public void setVelocity(Double velocity) {
        kicker.set(ControlMode.Velocity, velocity);
        System.out.println(velocity);
    }
}
