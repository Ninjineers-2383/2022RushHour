package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // create motor instance using a VictorSPX motor controller
    private final TalonSRX intakeMotor;

    // creates two solenoid instances
    private final Solenoid upSolenoid;
    private final Solenoid downSolenoid;

    /**
     * Intake subsystem constructor
     * 
     * @param compressor       the instance of the compressor subsystem
     * @param motorPort        the port of the intake motor
     * @param upSolenoidPort   the port of the up solenoid
     * @param downSolenoidPort the port of the down solenoid
     */
    public IntakeSubsystem(int motorPort, int upSolenoidPort, int downSolenoidPort) {
        intakeMotor = new TalonSRX(motorPort);
        upSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, upSolenoidPort);
        downSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, downSolenoidPort);

        intakeMotor.setInverted(true);
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
