package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class DrivetrainVoltTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DrivetrainSubsystem drivetrainSubsystem;
  private DoubleSupplier leftVolts;
  private DoubleSupplier rightVolts;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainVoltTest(DrivetrainSubsystem subsystem, DoubleSupplier leftVolts, DoubleSupplier rightVolts) {
    drivetrainSubsystem = subsystem;
    this.leftVolts = leftVolts;
    this.rightVolts = rightVolts;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // See DriveTrainSubsystem.java for more details how the arcade() method works.
    drivetrainSubsystem.tankDriveVolts(leftVolts.getAsDouble(), rightVolts.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // RerightVoltss true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
