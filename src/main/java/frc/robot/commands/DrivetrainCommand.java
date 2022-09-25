package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Creates an instance of the drivetrain
    private final DrivetrainSubsystem drivetrain;

    // This is the forward parameter
    private DoubleSupplier throttle;

    // This is the turn parameter
    private DoubleSupplier turn;

    /**
     * Takes in a drivetrain subsystem and forward and turn inputs for drive
     *
     * @param subsystem The subsystem used by this command.
     */
    public DrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplier throttle, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.throttle = throttle;
        this.turn = turn;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See drivetrain.java for more details how the arcade() method works.
        drivetrain.drive(throttle.getAsDouble(), turn.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    // Stops the drivetrain once the command ends
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
