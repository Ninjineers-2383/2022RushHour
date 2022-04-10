package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DrivetrainSubsystem drivetrainSubsystem;
    private DoubleSupplier throttle;
    private DoubleSupplier turn;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DrivetrainCommand(DrivetrainSubsystem subsystem, DoubleSupplier throttle, DoubleSupplier turn) {
        drivetrainSubsystem = subsystem;
        this.throttle = throttle;
        this.turn = turn;
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
        // See DriveTrainSubsystem.java for more details how the arcade() method works.
        drivetrainSubsystem.drive(throttle.getAsDouble(), turn.getAsDouble());
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
