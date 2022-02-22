package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.Intake;


public class IntakeSubsystem extends SubsystemBase {
    public  final VictorSPX frontMotor = new VictorSPX(Intake.FRONT_INTAKE_PORT);
    public  final VictorSPX rearMotor = new VictorSPX(Intake.REAR_INTAKE_PORT);

    public final Compressor pump = new Compressor(PneumaticsModuleType.CTREPCM);

    private final Solenoid frontUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_LEFT_SOLENOID_PORT);
    private final Solenoid rearUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_LEFT_SOLENOID_PORT);
    private final Solenoid frontDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_RIGHT_SOLENOID_PORT);
    private final Solenoid rearDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_RIGHT_SOLENOID_PORT);
    private boolean frontDown = false;
    private boolean rearDown = false;
    

    public IntakeSubsystem() {
        frontMotor.setInverted(false);
        rearMotor.setInverted(false);
        pump.enableDigital();
    }

    // deployed intakes turn on
    public void setPower(Double power) {
        if (frontDown) {
            frontMotor.set(ControlMode.PercentOutput, power);
        } else {
            frontMotor.set(ControlMode.PercentOutput, 0);
        }
        if (rearDown) {
            rearMotor.set(ControlMode.PercentOutput, power);
        } else {
            rearMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    // solenoid control
    public void setFrontDown(Boolean down) {
        frontUpSolenoid.set(down);
        frontDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Front Left Feeder State", frontUpSolenoid.get());
        SmartDashboard.putBoolean("Front Right Feeder State", frontDownSolenoid.get());
        rearDown = down;
    }

    
    public void setRearDown(Boolean down) {
        rearUpSolenoid.set(down);
        rearDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Rear Left Feeder State", rearUpSolenoid.get());
        SmartDashboard.putBoolean("Rear Right Feeder State", rearDownSolenoid.get());
        frontDown = down;
    }

    public boolean getFrontDown() {
        return frontDown;
    }

    public boolean getRearDown() {
        return rearDown;
    }

    public boolean getFrontLeftDown() {
        return frontUpSolenoid.get();

    }

    public boolean getFrontRightDown() {
        return frontDownSolenoid.get();
    }

    public boolean getRearRightDown() {
        return rearDownSolenoid.get();
    }

    public boolean getRearLeftDown() {
        return rearUpSolenoid.get();
    }
}