package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystemNew extends SubsystemBase {

    private final CANSparkMax right_climber = new CANSparkMax(Constants.Climber.RIGHT_PORT, MotorType.kBrushless);
    private final CANSparkMax left_climber = new CANSparkMax(Constants.Climber.LEFT_PORT, MotorType.kBrushless);

    private final RelativeEncoder left_encoder;
    private final RelativeEncoder right_encoder;
    private boolean leftOffToggle = false;
    private SlewRateLimiter positioning = new SlewRateLimiter(40);

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
        double kP = 0.1;
        double error = right_encoder.getPosition() - left_encoder.getPosition();
        double leftPower = leftOffToggle ? 0 : left * 0.97;
        left_climber.set(leftPower + MathUtil.clamp(error * kP, -0.1, 0.1));
        right_climber.set(right - MathUtil.clamp(error * kP, -0.1, 0.1));
    }

    public void resetPosition() {
        positioning.reset(0);
    }

    public boolean runToPosition(double targetPosition, boolean smooth) {
        double kP = 0.1;
        double DEADZONE = 0.5;
        double error;
        if (smooth) {
            error = positioning.calculate(targetPosition - getAverageEncoderPos());
        } else {
            error = targetPosition - getAverageEncoderPos();
        }

        if (Math.abs(error) > DEADZONE) {
            double speed = MathUtil.clamp(kP * error, -0.8, 0.8);
            setPower(speed, speed);
            return false;
        } else {
            setPower(0, 0);
            return true;
        }
    }

    public void switchBrakeCoast(boolean isBrake) {
        IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
        left_climber.setIdleMode(mode);
        right_climber.setIdleMode(mode);
    }

    public class SlewRateLimiter {
        private final double m_rateLimit;
        private double m_prevVal;
        private double m_prevTime;

        public SlewRateLimiter(double rateLimit, double initialValue) {
            m_rateLimit = rateLimit;
            m_prevVal = initialValue;
            m_prevTime = WPIUtilJNI.now() * 1e-6;
        }

        public SlewRateLimiter(double rateLimit) {
            this(rateLimit, 0);
        }

        public double calculate(double input) {
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            if (input < m_prevVal && m_prevVal > 0 || input > m_prevVal && m_prevVal < 0) {
                m_prevVal = input;
            }
            m_prevVal += MathUtil.clamp(input - m_prevVal, -m_rateLimit * elapsedTime, m_rateLimit * elapsedTime);
            m_prevTime = currentTime;
            return m_prevVal;
        }

        public void reset(double value) {
            m_prevVal = value;
            m_prevTime = WPIUtilJNI.now() * 1e-6;
        }
    }
}
