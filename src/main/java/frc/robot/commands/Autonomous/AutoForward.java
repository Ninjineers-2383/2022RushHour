package frc.robot.commands.Autonomous;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class AutoForward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double kP_HEADING_CORRECTION = 10;                                       // Strength of heading correction 
  private final double ADJUSTED_MAX_VOLTAGE;

  
  final private double DISTANCE_TICKS;
  final private double ACCELERATION_INTERVAL;
  final private double TIMEOUT;
  final private Timer TIMER;
  
  private boolean done = false;
  private int profileState = 0; //Finite State Machine

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoForward(DrivetrainSubsystem subsystem, double distanceFeet, double accelerationIntervalFeet, double maxVoltage, double timeout) {
    drivetrainSubsystem = subsystem;
    this.ADJUSTED_MAX_VOLTAGE = maxVoltage - Math.signum(maxVoltage) * Drivetrain.ksVolts;                      // Friction

    this.DISTANCE_TICKS = (int) (distanceFeet * Drivetrain.TICKS_PER_FOOT);
    this.ACCELERATION_INTERVAL = (int) (accelerationIntervalFeet * Drivetrain.TICKS_PER_FOOT);

    drivetrainSubsystem.zeroEncoders();
    this.TIMEOUT = timeout;
    this.TIMER = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Auto Done", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    double startHeading = drivetrainSubsystem.getHeading();
    double leftOutput = 0;
    double rightOutput = 0;

    if (TIMER.get() > TIMEOUT) {
      profileState = 3;
    }


    switch (profileState) {                                                      // Piece-wise motion profile
      case 0: // Ramp Up
        final double aL = drivetrainSubsystem.getLeftPosition() / (double) ACCELERATION_INTERVAL;
        final double aR = drivetrainSubsystem.getRightPosition() / (double) ACCELERATION_INTERVAL;

        leftOutput = aL;
        rightOutput = aR;

        if (drivetrainSubsystem.getLeftPosition() > ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }
      
      case 1: // Max Voltage
        leftOutput = 1;
        rightOutput = 1;
        if (drivetrainSubsystem.getLeftPosition() > DISTANCE_TICKS - ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }

      case 2: // Ramp Down
        final double bL = 1 - ((double) (drivetrainSubsystem.getLeftPosition() - DISTANCE_TICKS + ACCELERATION_INTERVAL)) / (double) ACCELERATION_INTERVAL;
        final double bR = 1 - ((double) (drivetrainSubsystem.getRightPosition() - DISTANCE_TICKS + ACCELERATION_INTERVAL)) / (double) ACCELERATION_INTERVAL;

        leftOutput = bL;
        rightOutput = bR;

        if (drivetrainSubsystem.getLeftPosition() >= DISTANCE_TICKS && drivetrainSubsystem.getRightPosition() >= DISTANCE_TICKS) {
          profileState ++;
        } else {
          break;
        }

      case 3:
        done = true;
    }
    
    leftOutput += kP_HEADING_CORRECTION * (drivetrainSubsystem.m_gyro.getAngle() - startHeading);                // Compensation for unwanted turn
    rightOutput -= kP_HEADING_CORRECTION * (drivetrainSubsystem.m_gyro.getAngle() - startHeading);

    leftOutput *= ADJUSTED_MAX_VOLTAGE;                                                   // Multiply by max voltage
    rightOutput *= ADJUSTED_MAX_VOLTAGE;

    leftOutput += Math.signum(ADJUSTED_MAX_VOLTAGE) * Drivetrain.ksVolts;
    rightOutput += Math.signum(ADJUSTED_MAX_VOLTAGE) * Drivetrain.ksVolts;
    drivetrainSubsystem.tankDriveVolts(leftOutput, rightOutput);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDriveVolts(0, 0);
    SmartDashboard.putBoolean("Auto Done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}