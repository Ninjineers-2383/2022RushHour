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
  private final double adjustedMaxOutput;

  
  final private double DISTANCE_TICKS;
  final private double ACCELERATION_INTERVAL;
  final private double TIMEOUT;
  final private Timer TIMER;

  final private double[] OFFSET_TICKS = new double[2];
  
  private boolean done = false;
  private int profileState = 0; //Finite State Machine

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoForward(DrivetrainSubsystem subsystem, double distanceFeet, double accelerationIntervalFeet, double maxOutput, double timeout) {
    drivetrainSubsystem = subsystem;
    this.adjustedMaxOutput = maxOutput - Math.signum(maxOutput) * (Drivetrain.ksPercent);                      // Friction

    this.DISTANCE_TICKS = (int) (distanceFeet * Drivetrain.TICKS_PER_FOOT);
    this.ACCELERATION_INTERVAL = (int) (accelerationIntervalFeet * Drivetrain.TICKS_PER_FOOT);

    OFFSET_TICKS[0] = drivetrainSubsystem.getLeftPosition();
    OFFSET_TICKS[1] = drivetrainSubsystem.getRightPosition();
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

    double workingTicks = (((drivetrainSubsystem.getLeftPosition() - OFFSET_TICKS[0]) + (drivetrainSubsystem.getRightPosition() - OFFSET_TICKS[1]))) / 2.0;

    if (TIMER.get() > TIMEOUT) {
      profileState = 3;
    }


    switch (profileState) {                                                      // Piece-wise motion profile
      case 0: // Ramp Up
        final double a = workingTicks / (double) ACCELERATION_INTERVAL;

        //SmartDashboard.putNumber("")

        leftOutput = a;
        rightOutput = a;

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
        final double b = 1 - ((double) (workingTicks - DISTANCE_TICKS + ACCELERATION_INTERVAL)) / (double) ACCELERATION_INTERVAL;

        leftOutput = b;
        rightOutput = b;

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

    leftOutput *= adjustedMaxOutput;                                                   // Multiply by max output
    rightOutput *= adjustedMaxOutput;

    leftOutput += Math.signum(adjustedMaxOutput) * (Drivetrain.ksPercent);
    rightOutput += Math.signum(adjustedMaxOutput) * (Drivetrain.ksPercent);

    // leftOutput /= 12;
    // rightOutput /= 12;

    SmartDashboard.putNumber("left", leftOutput);
    SmartDashboard.putNumber("right", rightOutput);

    drivetrainSubsystem.tankDrive(leftOutput, rightOutput);
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