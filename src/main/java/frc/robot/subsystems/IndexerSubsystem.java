package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;

public class IndexerSubsystem extends SubsystemBase {
    // create kicker instance that uses a TalonSRX motor controller.
    private final VictorSPX indexer = new VictorSPX(Kicker.PORT);

    // Kicker subsystem defined
    public IndexerSubsystem() {
        indexer.setInverted(false);
        indexer.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer Velocity", indexer.getSelectedSensorVelocity());
    }

    // method that returns nothing (void) but sets the kicker at a set power in the
    // interval of [-1, 1]
    public void setPower(Double power) {
        indexer.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Indexer power", power);
    }

    // method that returns nothing (void) but sets the kicker at a set velocity.
    public void setVelocity(Double velocity) {
        indexer.set(ControlMode.Velocity, velocity);
        System.out.println(velocity);
    }
}
