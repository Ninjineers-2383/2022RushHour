package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.Intake;


public class IntakeSubsystem extends SubsystemBase {
    public  final TalonSRX frontMotor = new TalonSRX(Intake.FRONT_INTAKE_PORT);
    public  final TalonSRX rearMotor = new TalonSRX(Intake.REAR_INTAKE_PORT);

    private final Solenoid frontLift = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.FRONT_SOLENOID_PORT);
    private final Solenoid rearLift = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.REAR_SOLENOID_PORT);
    

    public IntakeSubsystem() {
        frontMotor.setInverted(true);
        rearMotor.setInverted(false);
    }

    // deployed intakes turn on
    public void setPower(Double power) {
        if (getfrontDown()) {
            frontMotor.set(ControlMode.PercentOutput, power);
        }

        if (getRearDown()) {
            rearMotor.set(ControlMode.PercentOutput, power);
        }
    }

    // solenoid control
    public void setFrontDown(Boolean down) {
        frontLift.set(down);
        SmartDashboard.putBoolean("Front Feeder State", frontLift.get());
        
    }

    
    public void setRearDown(Boolean down) {
        rearLift.set(down);
        SmartDashboard.putBoolean("Rear Feeder State", rearLift.get());
    }


    public boolean getfrontDown() {
        return frontLift.get();
    }


    public boolean getRearDown() {
        return rearLift.get();
    }
}
