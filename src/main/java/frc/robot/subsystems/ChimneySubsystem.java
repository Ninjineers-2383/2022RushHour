package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Chimney;

public class ChimneySubsystem extends SubsystemBase {

    // create motor instance that uses a VictorSPX motor controller.
    private final VictorSPX motor;

    /**
     * Chimney subsystem constructor
     */
    public ChimneySubsystem() {
        motor = new VictorSPX(Chimney.PORT);
        motor.setInverted(true);

    }

    /**
     * Sets the power of the chimney from -1 to 1
     * 
     * @param power the power of the chimney
     */
    public void setPower(Double power) {
        motor.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Chimney Power", power);
    }
}
