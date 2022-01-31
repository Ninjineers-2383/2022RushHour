package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.Chimney;


public class ChimneySubsystem extends SubsystemBase {

    // create motor instance that uses a TalonSRX motor controller.
    private final TalonSRX motor;
    
    // Chimney subsystem constructor
    public ChimneySubsystem() {
        motor = new TalonSRX(Chimney.PORT);
        motor.setInverted(true);
        
    }

    // method that returns nothing (void) but sets the kicker at a set power in the inverval of [-1, 1]
    public void setPower(Double power) {
        motor.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Chimney Power", power);
    }
}