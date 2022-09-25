package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Defines instance of the launcher subsystem from LauncherSubsystem.java
    private final LauncherSubsystem m_subsystem;

    private final DoubleSupplier m_speed;

    private final BooleanSupplier m_shouldChangeSpeed;

    private final boolean m_stopOnEnd;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a launcher command that takes in the launcher subsystem and
    // runs launcher subsystem actions.
    public LauncherCommand(LauncherSubsystem subsystem, DoubleSupplier speed,
            BooleanSupplier shouldChangeSpeed) {
        this(subsystem, speed, shouldChangeSpeed, true);
    }

    /**
     * A launcher command that takes in the launcher subsystem and runs launcher
     * subsystem actions.
     * 
     * @param launcher          instance of launcher
     * @param speed             speed of launcher
     * @param shouldChangeSpeed whether or not the launcher should change its speed
     */
    public LauncherCommand(LauncherSubsystem subsystem, DoubleSupplier speed, BooleanSupplier shouldChangeSpeed,
            boolean stopOnEnd) {
        m_subsystem = subsystem;
        m_speed = speed;
        m_shouldChangeSpeed = shouldChangeSpeed;
        m_stopOnEnd = stopOnEnd;

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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (m_stopOnEnd) {
            m_subsystem.spin(0);
        }
    }
}
