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
  private final double ADJUSTED_MAX_POWER;

  
  final private int DISTANCE_TICKS;
  final private int ACCELERATION_INTERVAL;
  final private double TIMEOUT;
  final private Timer TIMER;
  final double startHeading;
  
  private boolean done = false;
  private int profileState = 0; //Finite State Machine

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoForward(DrivetrainSubsystem subsystem, double distanceFeet, double accelerationIntervalFeet, double maxVoltage, double timeout) {
    drivetrainSubsystem = subsystem;
    this.ADJUSTED_MAX_POWER = maxVoltage - Math.signum(maxVoltage) * Drivetrain.ksVolts;                      // Friction

    this.DISTANCE_TICKS = (int) (distanceFeet * Drivetrain.TICKS_PER_FOOT);
    this.ACCELERATION_INTERVAL = (int) (accelerationIntervalFeet * Drivetrain.TICKS_PER_FOOT);
    this.startHeading = drivetrainSubsystem.getHeading();

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
    
    double averageTicks = ((double) (drivetrainSubsystem.getLeftPosition() + drivetrainSubsystem.getRightPosition())) / 2;
    double output = 0;

    if (TIMER.get() > TIMEOUT) {
      profileState = 3;
    }


    switch (profileState) {                                                      // Piece-wise motion profile
      case 0: // Ramp Up
        final double a =  averageTicks / (double) ACCELERATION_INTERVAL;

        output = a;

        if (averageTicks > ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }
      
      case 1: // Max Voltage
        output = 1;
        if (averageTicks > DISTANCE_TICKS - ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }

      case 2: // Ramp Down
        final double b = 1 - (averageTicks - DISTANCE_TICKS + ACCELERATION_INTERVAL) / (double) ACCELERATION_INTERVAL;

        output = b;

        if (drivetrainSubsystem.getLeftPosition() >= DISTANCE_TICKS && drivetrainSubsystem.getRightPosition() >= DISTANCE_TICKS) {
          profileState ++;
        } else {
          break;
        }

      case 3:
        done = true;
    }
    
    double headingCorrection = kP_HEADING_CORRECTION * (drivetrainSubsystem.getHeading() - startHeading) ;                // Compensation for unwanted turn

    drivetrainSubsystem.tankDrive(
      (output + headingCorrection) * ADJUSTED_MAX_POWER + Math.signum(ADJUSTED_MAX_POWER) * Drivetrain.ksVolts,
     (output - headingCorrection) * ADJUSTED_MAX_POWER + Math.signum(ADJUSTED_MAX_POWER) * Drivetrain.ksVolts);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDrive(0, 0);
    SmartDashboard.putBoolean("Auto Done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
