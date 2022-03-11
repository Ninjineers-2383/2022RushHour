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
    public  final VictorSPX frontMotor = new VictorSPX(Intake.FRONT_INTAKE_PORT);
    public  final VictorSPX rearMotor = new VictorSPX(Intake.REAR_INTAKE_PORT);

    public final Compressor pump = new Compressor(PneumaticsModuleType.CTREPCM);

    public static final Solenoid frontUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_LEFT_SOLENOID_PORT);
    public static final Solenoid rearUpSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_LEFT_SOLENOID_PORT);
    private static final Solenoid frontDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_RIGHT_SOLENOID_PORT);
    private static final Solenoid rearDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_RIGHT_SOLENOID_PORT);
    private boolean frontDown = false;
    private boolean rearDown = false;
    

    public IntakeSubsystem() {
        frontMotor.setInverted(false);
        rearMotor.setInverted(false);
        pump.enableDigital();
    }

    // deployed intakes turn on
    public void setPower(Double power) {
        rearMotor.set(ControlMode.PercentOutput, power);
        frontMotor.set(ControlMode.PercentOutput, power);
        // if (frontDown) {
        // } else {
        //     frontMotor.set(ControlMode.PercentOutput, 0);
        // }
        // if (rearDown) {
        // } else {
        //     rearMotor.set(ControlMode.PercentOutput, 0);
        // }
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
        rearDown = down;
        SmartDashboard.putBoolean("rearDown", rearDown);
    }

    
    public void setRearDown(Boolean down) {
        rearUpSolenoid.set(down);
        rearDownSolenoid.set(!down);
        SmartDashboard.putBoolean("Rear Left Feeder State", rearUpSolenoid.get());
        SmartDashboard.putBoolean("Rear Right Feeder State", rearDownSolenoid.get());
        frontDown = down;
        SmartDashboard.putBoolean("frontDown", frontDown);
    }

    public boolean getFrontDown() {
        return frontDown;
    }

    public boolean getRearDown() {
        return rearDown;
    }

    public boolean getFrontUp() {
        return frontUpSolenoid.get();

    }

    public boolean getFrontRightDown() {
        return frontDownSolenoid.get();
    }

    public boolean getRearRightDown() {
        return rearDownSolenoid.get();
    }

    public boolean getRearUp() {
        return rearUpSolenoid.get();
    }
}
