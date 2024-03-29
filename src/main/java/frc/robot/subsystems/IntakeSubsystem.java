package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CompressorCommand;

public class IntakeSubsystem extends SubsystemBase {
    // create motor instance using a VictorSPX motor controller
    private final VictorSPX intakeMotor;

    // creates two solenoid instances
    private final Solenoid upSolenoid;
    private final Solenoid downSolenoid;

    // creates an instance of the compressor command
    public final CompressorCommand compressorCommand;

    /**
     * Intake subsystem constructor
     * 
     * @param compressor       the instance of the compressor subsystem
     * @param motorPort        the port of the intake motor
     * @param upSolenoidPort   the port of the up solenoid
     * @param downSolenoidPort the port of the down solenoid
     */
    public IntakeSubsystem(CompressorSubsystem compressor, int motorPort, int upSolenoidPort, int downSolenoidPort) {
        this.compressorCommand = new CompressorCommand(compressor);
        intakeMotor = new VictorSPX(motorPort);
        upSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, upSolenoidPort);
        downSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, downSolenoidPort);

        intakeMotor.setInverted(false);
    }

    /**
     * Sets the power of the intake motor
     * 
     * @param power power of the intake motor from -1 to 1
     */
    public void setPower(Double power) {
        intakeMotor.set(ControlMode.PercentOutput, power * (getDown() ? 0.2 : 1));
    }

    /**
     * Sets the intake up or down
     * 
     * @param down true sets the intake down, false sets the intake up
     */
    public void setDown(Boolean down) {
        upSolenoid.set(down);
        downSolenoid.set(!down);
        if (down) {
            compressorCommand.useCompressor();
        } else {
            compressorCommand.stopUsingCompressor();
        }
    }

    /**
     * Gets the state of the intake
     * 
     * @return whether or not the intake is up
     */
    public boolean getUp() {
        boolean front = !upSolenoid.get();
        return front;
    }

    /**
     * Gets the state of the intake
     * 
     * @return whether or not the intake is down
     */
    public boolean getDown() {
        return !upSolenoid.get();
    }
}
