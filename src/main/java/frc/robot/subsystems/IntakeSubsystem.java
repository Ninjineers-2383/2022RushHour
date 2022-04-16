package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {
    private final VictorSPX frontMotor = new VictorSPX(Intake.FRONT_INTAKE_PORT);
    private final VictorSPX rearMotor = new VictorSPX(Intake.REAR_INTAKE_PORT);

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // public final DoubleSolenoid = new
    // DoubleSolenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_LEFT_SOLENOID_POR)
    public static final Solenoid frontUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            Intake.FRONT_LEFT_SOLENOID_PORT);
    public static final Solenoid rearUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            Intake.REAR_LEFT_SOLENOID_PORT);
    private static final Solenoid frontDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            Intake.FRONT_RIGHT_SOLENOID_PORT);
    private static final Solenoid rearDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            Intake.REAR_RIGHT_SOLENOID_PORT);

    public IntakeSubsystem() {
        // frontMotor.setSafetyEnabled();
        frontMotor.setInverted(false);
        rearMotor.setInverted(false);
        enableCompressor(true);
    }

    // deployed intakes turn on
    public void setPower(Double power) {
        rearMotor.set(ControlMode.PercentOutput, power * (getFrontDown() ? 0.2 : 1));
        frontMotor.set(ControlMode.PercentOutput, power * (getRearDown() ? 0.2 : 1));
    }

    public void setPower2(Double frontPower, Double rearPower) {
        frontMotor.set(ControlMode.PercentOutput, frontPower);
        rearMotor.set(ControlMode.PercentOutput, rearPower);
    }

    // solenoid control
    public void setFrontDown(Boolean down) {
        frontUpSolenoid.set(down);
        frontDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Front Left Feeder State", frontUpSolenoid.get());
        SmartDashboard.putBoolean("Front Right Feeder State", frontDownSolenoid.get());
        SmartDashboard.putBoolean("rearDown", down);
    }

    public void setRearDown(Boolean down) {
        rearUpSolenoid.set(down);
        rearDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Rear Left Feeder State", rearUpSolenoid.get());
        SmartDashboard.putBoolean("Rear Right Feeder State", rearDownSolenoid.get());
        SmartDashboard.putBoolean("frontDown", down);
    }

    public boolean getFrontUp() {
        boolean front = !frontUpSolenoid.get();
        SmartDashboard.putBoolean("get front up", front);
        return front;
    }

    public boolean getFrontDown() {
        return frontDownSolenoid.get();
    }

    public boolean getRearDown() {
        return rearDownSolenoid.get();
    }

    public boolean getRearUp() {
        boolean rear = !rearUpSolenoid.get();
        SmartDashboard.putBoolean("get rear up", rear);
        return rear;
    }

    public void enableCompressor(boolean enable) {
        if (enable) {
            compressor.enableDigital();
        } else {
            compressor.disable();
        }
    }
}
