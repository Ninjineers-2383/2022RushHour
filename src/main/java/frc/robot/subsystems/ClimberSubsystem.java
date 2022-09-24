package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    // creates two motor instances with a SparkMax motor controller
    private final CANSparkMax right_climber = new CANSparkMax(Constants.Climber.RIGHT_PORT, MotorType.kBrushless);
    private final CANSparkMax left_climber = new CANSparkMax(Constants.Climber.LEFT_PORT, MotorType.kBrushless);

    // creates two encoder instances
    private final RelativeEncoder left_encoder;
    private final RelativeEncoder right_encoder;

    // toggles the left motor off (true) and on (false)
    private boolean leftOffToggle;

    /**
     * Climber subsystem constructor
     */
    public ClimberSubsystem() {
        right_climber.setInverted(false);
        left_climber.setInverted(true);

        right_climber.setIdleMode(IdleMode.kBrake);
        left_climber.setIdleMode(IdleMode.kBrake);

        left_encoder = left_climber.getEncoder();
        right_encoder = right_climber.getEncoder();

        left_encoder.setPosition(0);
        right_encoder.setPosition(0);

        leftOffToggle = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber pos", left_encoder.getPosition());
        SmartDashboard.putNumber("Right Climber pos", right_encoder.getPosition());
        SmartDashboard.putBoolean("leftClimberOff", leftOffToggle);
    }

    /**
     * Toggles the left motor on and off
     */
    public void leftMotorToggle() {
        leftOffToggle = !leftOffToggle;
    }

    /**
     * Gets the left encoder position
     */
    public double getLeftEncoderPos() {
        return left_encoder.getPosition();
    }

    /**
     * Gets the right encoder position
     */
    public double getRightEncoderPos() {
        return right_encoder.getPosition();
    }

    /**
     * Gets the average of the left and right encoder position
     */
    public double getAverageEncoderPos() {
        return (getLeftEncoderPos() + getRightEncoderPos()) / 2;
    }

    /**
     * Sets the power of the left and right encoders
     * 
     * @param left  left motor power
     * @param right right motor power
     */
    public void setPower(double left, double right) {
        double leftPower = leftOffToggle ? 0 : left * 0.97;
        left_climber.set(leftPower);
        right_climber.set(right);
    }

    /**
     * Switches the climbers between coast and brake mode
     * 
     * @param isBrake true sets the climbers to brake and false sets the climbers to
     *                coast
     */
    public void switchBrakeCoast(boolean isBrake) {
        IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
        left_climber.setIdleMode(mode);
        right_climber.setIdleMode(mode);
    }
}
