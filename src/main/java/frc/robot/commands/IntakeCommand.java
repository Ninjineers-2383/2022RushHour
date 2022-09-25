package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final IntakeSubsystem intake;
    private DoubleSupplier power;
    private boolean down;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a feeder command that takes in the feeder subsystem and runs
    // feeder subsystem actions.
    /**
     * A feeder command that takes in the feeder subsystem and runs feeder subsystem
     * actions.
     * 
     * @param intake the instance of the feeder
     * @param power  the power of the intake motors
     * @param down   whether or not the intakes are down
     */
    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier power, boolean down) {
        this.intake = intake;
        this.power = power;
        this.down = down;
        addRequirements(intake);
    }

    // Called once when the command gets scheduled
    @Override
    public void initialize() {
        intake.setDown(down);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See FeederSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        double d_power = power.getAsDouble();
        intake.setDown(Math.abs(d_power) > 0.2);
        intake.setPower(d_power);
    }

    // Called once when the command ends
    @Override
    public void end(boolean interrupted) {
        intake.setDown(false);
        intake.setPower(0.0);
    }
}
