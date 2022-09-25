package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Defines instance of the launcher subsystem from LauncherSubsystem.java
    private final LauncherSubsystem launcher;

    private final DoubleSupplier speed;

    private final BooleanSupplier shouldChangeSpeed;

    private double previousSpeed;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a launcher command that takes in the launcher subsystem and
    // runs launcher subsystem actions.

    /**
     * A launcher command that takes in the launcher subsystem and runs launcher
     * subsystem actions.
     * 
     * @param launcher          instance of launcher
     * @param speed             speed of launcher
     * @param shouldChangeSpeed whether or not the launcher should change its speed
     */
    public LauncherCommand(LauncherSubsystem launcher, DoubleSupplier speed, BooleanSupplier shouldChangeSpeed) {
        this.launcher = launcher;
        this.speed = speed;
        this.shouldChangeSpeed = shouldChangeSpeed;
        previousSpeed = Double.NaN;
        addRequirements(launcher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // see LauncherSubsystem.java for more details on how spin() method works
        if (shouldChangeSpeed.getAsBoolean()) {
            double d_speed = speed.getAsDouble();
            if (d_speed == previousSpeed) {
                return;
            }
            launcher.spin(d_speed);
            previousSpeed = d_speed;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        launcher.spin(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
