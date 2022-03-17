package frc.robot.commands.Autonomous;

import java.util.function.DoubleSupplier;

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
  DoubleSupplier forward;
  DoubleSupplier turn;
  double firstTurn = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(DrivetrainSubsystem drivetrain, CameraSubsystem camera) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.forward = () -> 0;
    this.turn = () -> 0;
    this.firstTurn = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  public AutoAlign(DrivetrainSubsystem drivetrain, CameraSubsystem camera, DoubleSupplier forward,
      DoubleSupplier turn) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.forward = forward;
    this.turn = turn;
    this.firstTurn = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  public AutoAlign(DrivetrainSubsystem drivetrain, CameraSubsystem camera, double firstTurn) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.forward = () -> 0;
    this.turn = () -> 0;
    this.firstTurn = firstTurn;

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
    double kp = 0.005;
    if (firstTurn == 0) {
      drivetrain.drive(forward.getAsDouble(),
          ((camera.getValid() && (Math.abs(camera.getX()) > 1) ? 1 : 0))
              * (kp * camera.getX() + Math.signum(camera.getX()) * 0.3) + turn.getAsDouble());
    } else {
      if (camera.getValid()) {
        if (Math.abs(camera.getX()) > 2) {
          drivetrain.drive(0,
              (kp * camera.getX() + Math.signum(camera.getX()) * 0.3));
        } else {
          done = true;
        }

      } else {
        drivetrain.drive(0, firstTurn);
      }

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
