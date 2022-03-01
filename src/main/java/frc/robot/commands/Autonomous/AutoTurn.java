package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class AutoTurn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double ADJUSTED_MAX_POWER;

  
  final private double TARGET_HEADING;
  final private double ACCELERATION_INTERVAL;
  final private double TIMEOUT;
  final private Timer TIMER;
  
  private boolean done = false;
  double startHeading;
  private int profileState = 0; //Finite State Machine


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoTurn(DrivetrainSubsystem subsystem, double targetHeading, double accelerationInterval, double maxPower, double timeout) {
    drivetrainSubsystem = subsystem;
    this.ADJUSTED_MAX_POWER = maxPower - Math.signum(maxPower) * Drivetrain.ksPercentTurn;                      // Friction

    this.TARGET_HEADING = targetHeading;
    this.ACCELERATION_INTERVAL = accelerationInterval;

    this.TIMEOUT = timeout;
    this.TIMER = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Auto Done", false);
    startHeading = drivetrainSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offsetHeading = Math.abs(drivetrainSubsystem.getHeading() - startHeading);
    double output = 0;

    if (TIMER.get() > TIMEOUT) {
      profileState = 3;
    }

    switch (profileState) {  // Piece-wise Trapezoidal motion profile
      case 0: // Ramp Up
        final double a =  offsetHeading / ACCELERATION_INTERVAL;

        output = a;

        if (offsetHeading > ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }
      
      case 1: // Max Voltage
        output = 1;
        if (offsetHeading > TARGET_HEADING - ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }

      case 2: // Ramp Down
        final double b = ((offsetHeading - TARGET_HEADING + ACCELERATION_INTERVAL)) / (ACCELERATION_INTERVAL);

        output = (1 - b);

        if (offsetHeading >= TARGET_HEADING) {
          profileState ++;
        } else {
          break;
        }

      case 3: // Finish Auto
        done = true;
    }

    output *= ADJUSTED_MAX_POWER;                                                   // Multiply by max voltage

    output += Math.signum(ADJUSTED_MAX_POWER) * Drivetrain.ksPercentTurn;
    drivetrainSubsystem.tankDrive(-output, output);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Auto Done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
