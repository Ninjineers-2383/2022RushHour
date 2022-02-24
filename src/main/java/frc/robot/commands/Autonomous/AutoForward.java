package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  final private double OFFSET_TICKS;
  
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

    OFFSET_TICKS = (drivetrainSubsystem.getLeftPosition() + drivetrainSubsystem.getRightPosition()) / 2.0;
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
    System.out.println(profileState);

    double startHeading = drivetrainSubsystem.getHeading();
    double leftOutput = 0;
    double rightOutput = 0;

    double workingTicks = ((drivetrainSubsystem.getLeftPosition() + drivetrainSubsystem.getRightPosition()) / 2.0) - OFFSET_TICKS;

    if (TIMER.get() > TIMEOUT) {
      profileState = 3;
    }


    switch (profileState) {                                                      // Piece-wise motion profile
      case 0: // Ramp Up
        final double aL = workingTicks / (double) ACCELERATION_INTERVAL;
        final double aR = workingTicks / (double) ACCELERATION_INTERVAL;

        leftOutput = aL;
        rightOutput = aR;

        if (workingTicks > ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }
      
      case 1: // Max Voltage
        leftOutput = 1;
        rightOutput = 1;
        if (workingTicks > DISTANCE_TICKS - ACCELERATION_INTERVAL) {
          profileState ++;
        } else {
          break;
        }

      case 2: // Ramp Down
        final double bL = 1 - ((double) (workingTicks - DISTANCE_TICKS + ACCELERATION_INTERVAL)) / (double) ACCELERATION_INTERVAL;
        final double bR = 1 - ((double) (workingTicks - DISTANCE_TICKS + ACCELERATION_INTERVAL)) / (double) ACCELERATION_INTERVAL;

        leftOutput = bL;
        rightOutput = bR;

        if (workingTicks >= DISTANCE_TICKS) {
          profileState ++;
        } else {
          break;
        }

      case 3:
        done = true;
    }
    
    leftOutput += kP_HEADING_CORRECTION * (drivetrainSubsystem.getHeading() - startHeading);                // Compensation for unwanted turn
    rightOutput -= kP_HEADING_CORRECTION * (drivetrainSubsystem.getHeading() - startHeading);

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