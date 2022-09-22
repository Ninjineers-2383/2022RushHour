package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CompressorCommand;

public class IntakeSubsystem extends SubsystemBase {
    private final VictorSPX intakeMotor;

    private final Solenoid upSolenoid;
    private final Solenoid downSolenoid;

    private final CompressorSubsystem compressor;
    private final CompressorCommand compressorCommand;

    public IntakeSubsystem(CompressorSubsystem compressor, int motorPort, int upSolenoidPort, int downSolenoidPort) {
        this.compressor = compressor;
        this.compressorCommand = new CompressorCommand(compressor);
        intakeMotor = new VictorSPX(motorPort);
        upSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, upSolenoidPort);
        downSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, downSolenoidPort);

        intakeMotor.setInverted(false);
    }

    // deployed intakes turn on
    public void setPower(Double power) {
        intakeMotor.set(ControlMode.PercentOutput, power * (getDown() ? 0.2 : 1));
    }

    // solenoid control
    public void setDown(Boolean down) {
        upSolenoid.set(down);
        downSolenoid.set(!down);
        if (down) {
            compressorCommand.useCompressor();
        } else {
            compressorCommand.stopUsingCompressor();
        }
    }

    public boolean getUp() {
        boolean front = !upSolenoid.get();
        return front;
    }

    public boolean getDown() {
        return !upSolenoid.get();
    }
}
