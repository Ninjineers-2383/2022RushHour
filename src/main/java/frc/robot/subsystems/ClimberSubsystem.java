package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax right_climber = new CANSparkMax(Constants.Climber.RIGHT_PORT, MotorType.kBrushless);
    private final CANSparkMax left_climber = new CANSparkMax(Constants.Climber.LEFT_PORT, MotorType.kBrushless);

    private final RelativeEncoder left_encoder;
    private final RelativeEncoder right_encoder;
    private final int RIGHT_LIMIT = 159;
    private final int LEFT_LIMIT = 157;
    private final double PASSIVE_SPEED = -0.2;

    // Climber subsystem constructor
    public ClimberSubsystem() {
        right_climber.setInverted(true);
        left_climber.setInverted(true);

        right_climber.setIdleMode(IdleMode.kBrake);
        left_climber.setIdleMode(IdleMode.kBrake);

        left_encoder = left_climber.getEncoder();
        right_encoder = right_climber.getEncoder();

        left_encoder.setPosition(0);
        right_encoder.setPosition(0);

        left_climber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        right_climber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        left_climber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LEFT_LIMIT);
        right_climber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, RIGHT_LIMIT);

        left_climber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        right_climber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        left_climber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -5);
        right_climber.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -5);
    }

    public void invertMotorPowers() {
        right_climber.setInverted(false);
        left_climber.setInverted(false);
    }

    public void unInvertMotorPower() {
        right_climber.setInverted(true);
        left_climber.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber pos", left_encoder.getPosition());
        SmartDashboard.putNumber("Right Climber pos", right_encoder.getPosition());
        if (left_encoder.getPosition() > LEFT_LIMIT) {
            left_climber.set(PASSIVE_SPEED);
        }
        if (right_encoder.getPosition() > RIGHT_LIMIT) {
            right_climber.set(PASSIVE_SPEED);
        }
    }

    public void setPower(double left, double right, double hookPower) {
        if (left_encoder.getPosition() < 3) {
            left_climber.set(left * 0.5);
            right_climber.set(right * 0.5);
        } else {
            left_climber.set(left);
            right_climber.set(right);
        }
    }

    public void switchBrakeCoast(boolean isBrake) {
        IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
        left_climber.setIdleMode(mode);
        right_climber.setIdleMode(mode);
    }
}
