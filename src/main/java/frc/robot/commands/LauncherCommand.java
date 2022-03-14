package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  // Defines instance of the launcher subsystem from LauncherSubsystem.java
  private final LauncherSubsystem m_subsystem;

  private final DoubleSupplier m_speed;

  private final BooleanSupplier m_shouldChangeSpeed;

  double timeOut;

  Timer timer = new Timer();

  // Creates a command that takes in a subsystem and speed and runs specific
  // actions created in the subsystem.
  // In this case, a launcher command that takes in the launcher subsystem and
  // runs launcher subsystem actions.
  public LauncherCommand(LauncherSubsystem subsystem, DoubleSupplier speed) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_shouldChangeSpeed = () -> true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public LauncherCommand(LauncherSubsystem subsystem, DoubleSupplier speed, BooleanSupplier shouldChangeSpeed) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_shouldChangeSpeed = shouldChangeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public LauncherCommand(LauncherSubsystem subsystem, DoubleSupplier speed, double timeOut) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_shouldChangeSpeed = () -> true;
    timer.reset();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // see LauncherSubsystem.java for more details on how spin() method works
    if (m_shouldChangeSpeed.getAsBoolean()) {
      m_subsystem.spin(m_speed.getAsDouble());
    }
  }

  public double speed() {
    // a public method that returns the speed as a double
    return m_speed.getAsDouble();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
