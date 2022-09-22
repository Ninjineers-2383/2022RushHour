package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final IntakeSubsystem intake;
    private BooleanSupplier power;
    private boolean down;
    private boolean movePistons;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a feeder command that takes in the feeder subsystem and runs
    // feeder subsystem actions.
    public IntakeCommand(IntakeSubsystem intake, BooleanSupplier power, boolean down, boolean movePistons) {
        this.intake = intake;
        this.power = power;
        this.down = down;
        this.movePistons = movePistons;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (movePistons) {
            intake.setDown(down);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See FeederSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        if (power.getAsBoolean()) {
            intake.setDown(true);
            intake.setPower(-1.0);
        } else {
            intake.setDown(false);
            intake.setPower(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setDown(false);
        intake.setPower(0.0);
    }
}
