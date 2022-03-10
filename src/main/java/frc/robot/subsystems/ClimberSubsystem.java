package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    private final TalonSRX hook = new TalonSRX(Constants.Climber.HOOK_PORT);

    private final RelativeEncoder left_encoder;
    private final RelativeEncoder right_encoder;
    private final int RIGHTLIMIT = 159;
    private final int LEFTLIMIT = 157;
    private final double PASSIVESPEED = -0.2;
    
    // Climber subsystem constructor
    public ClimberSubsystem() {
        right_climber.setInverted(true);
        left_climber.setInverted(true);
        hook.setInverted(false);

        right_climber.setIdleMode(IdleMode.kBrake);
        left_climber.setIdleMode(IdleMode.kBrake);
        hook.setNeutralMode(NeutralMode.Brake);

        left_encoder = left_climber.getEncoder();
        right_encoder = right_climber.getEncoder();

        left_encoder.setPosition(0);
        right_encoder.setPosition(0);

        left_climber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        right_climber.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        left_climber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LEFTLIMIT);
        right_climber.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, RIGHTLIMIT);

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
        if (left_encoder.getPosition() > LEFTLIMIT) {
            left_climber.set(PASSIVESPEED);
        }
        if (right_encoder.getPosition() > RIGHTLIMIT) {
            right_climber.set(PASSIVESPEED);
        }
    }

    public void setPower(double left, double right, double hookPower) {
        hook.set(ControlMode.PercentOutput, hookPower);
        if(left_encoder.getPosition() < 3) {
            left_climber.set(left * 0.5);
            right_climber.set(right * 0.5);
        }
        else {
            left_climber.set(left);
            right_climber.set(right);
        }
    }

    public void switchBrakeCoast(boolean isBrake) {
        IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
        left_climber.setIdleMode(mode);
        right_climber.setIdleMode(mode);
        hook.setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
      }
}
