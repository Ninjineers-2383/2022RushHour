package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.Intake;


public class IntakeSubsystem extends SubsystemBase {
    public  final TalonSRX frontMotor = new TalonSRX(Intake.FRONT_INTAKE_PORT);
    public  final TalonSRX rearMotor = new TalonSRX(Intake.REAR_INTAKE_PORT);

    public final Compressor pump = new Compressor(PneumaticsModuleType.CTREPCM);

    private final Solenoid frontUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_LEFT_SOLENOID_PORT);
    private final Solenoid rearUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_LEFT_SOLENOID_PORT);
    private final Solenoid frontDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_RIGHT_SOLENOID_PORT);
    private final Solenoid rearDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_RIGHT_SOLENOID_PORT);
    

    public IntakeSubsystem() {
        frontMotor.setInverted(true);
        rearMotor.setInverted(false);
        pump.enableDigital();
    }

    // deployed intakes turn on
    public void setPower(Double power) {
        frontMotor.set(ControlMode.PercentOutput, power);
        rearMotor.set(ControlMode.PercentOutput, power);
    }

    // solenoid control
    public void setFrontDown(Boolean down) {
        frontUpSolenoid.set(down);
        frontDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Front Left Feeder State", frontUpSolenoid.get());
        SmartDashboard.putBoolean("Front Right Feeder State", frontDownSolenoid.get());
        
    }

    
    public void setRearDown(Boolean down) {
        rearUpSolenoid.set(down);
        rearDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Rear Left Feeder State", rearUpSolenoid.get());
        SmartDashboard.putBoolean("Rear Right Feeder State", rearDownSolenoid.get());
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
