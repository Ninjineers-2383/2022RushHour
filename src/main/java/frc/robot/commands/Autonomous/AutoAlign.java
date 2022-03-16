package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAlign extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final DrivetrainSubsystem drivetrain;
  private final CameraSubsystem camera;

  private boolean done = false;
  double startHeading;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(DrivetrainSubsystem drivetrain, CameraSubsystem camera) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (camera.getValid()) {
      if (camera.getX() > 10) {
        drivetrain.drive(0, 0.3);
      } else if (camera.getX() < -10) {
        drivetrain.drive(0, -0.3);
      } else {
        drivetrain.drive(0, 0);
      }
    } else {
      drivetrain.drive(0, 0);
    }
    drivetrain.driveFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
