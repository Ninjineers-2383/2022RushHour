package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KickerSubsystem;

public class KickerCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Creates an instance of the kicker
    private final KickerSubsystem kicker;

    // The power of the kicker motors
    private final DoubleSupplier power;

    /**
     * A kicker command that takes in the kicker subsystem and runs
     * kicker subsystem actions.
     * 
     * @param kicker instance of kicker
     * @param power  power of kicker
     */
    public KickerCommand(KickerSubsystem kicker, DoubleSupplier power) {
        this.kicker = kicker;
        this.power = power;
        addRequirements(kicker);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See KickerSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        double d_power = power.getAsDouble();
        kicker.setPower(d_power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        kicker.setPower(0.0);
    }
}
