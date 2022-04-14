package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystemNew extends SubsystemBase {

    private final CANSparkMax right_climber = new CANSparkMax(Constants.Climber.RIGHT_PORT, MotorType.kBrushless);
    private final CANSparkMax left_climber = new CANSparkMax(Constants.Climber.LEFT_PORT, MotorType.kBrushless);

    private final RelativeEncoder left_encoder;
    private final RelativeEncoder right_encoder;
    private boolean leftOffToggle = false;;

    // Climber subsystem constructor
    public ClimberSubsystemNew() {
        right_climber.setInverted(false);
        left_climber.setInverted(true);

        right_climber.setIdleMode(IdleMode.kBrake);
        left_climber.setIdleMode(IdleMode.kBrake);

        left_encoder = left_climber.getEncoder();
        right_encoder = right_climber.getEncoder();

        left_encoder.setPosition(0);
        right_encoder.setPosition(0);
    }

    public void invertMotorPowers() {
        leftOffToggle = !leftOffToggle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber pos", left_encoder.getPosition());
        SmartDashboard.putNumber("Right Climber pos", right_encoder.getPosition());
        SmartDashboard.putBoolean("leftClimberOff", leftOffToggle);
        // if (Math.abs(right_encoder.getPosition() - left_encoder.getPosition()) > 10)
        // {
        // if (right_encoder.getPosition() > left_encoder.getPosition()) {
        // setPower(0, -0.6);
        // } else {
        // setPower(-0.6, 0);
        // }
        // }
    }

    public double getLeftEncoderPos() {
        return left_encoder.getPosition();
    }

    public double getRightEncoderPos() {
        return right_encoder.getPosition();
    }

    public double getAverageEncoderPos() {
        return (getLeftEncoderPos() + getRightEncoderPos()) / 2;
    }

    public void setPower(double left, double right) {
        double leftPower = leftOffToggle ? 0 : left * 0.97;
        left_climber.set(leftPower);
        right_climber.set(right);
    }

    public void switchBrakeCoast(boolean isBrake) {
        IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
        left_climber.setIdleMode(mode);
        right_climber.setIdleMode(mode);
    }
}
