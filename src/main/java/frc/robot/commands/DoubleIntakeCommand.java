package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DoubleIntakeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final IntakeSubsystem intake;
    private DoubleSupplier frontPower;
    private DoubleSupplier rearPower;
    private boolean frontDown;
    private boolean rearDown;
    private boolean movePistons = true;

    private double previousFrontPower = Double.NaN;
    private double previousRearPower = Double.NaN;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a feeder command that takes in the feeder subsystem and runs
    // feeder subsystem actions.
    public DoubleIntakeCommand(IntakeSubsystem intake, DoubleSupplier frontPower, DoubleSupplier rearPower,
            boolean frontDown, boolean rearDown) {
        this.intake = intake;
        this.frontPower = frontPower;
        this.rearPower = rearPower;
        this.frontDown = frontDown;
        this.rearDown = rearDown;
        addRequirements(intake);
    }

    public DoubleIntakeCommand(IntakeSubsystem intake, DoubleSupplier frontPower, DoubleSupplier rearPower) {
        this.intake = intake;
        this.frontPower = frontPower;
        this.rearPower = rearPower;
        this.movePistons = false;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (movePistons) {
            intake.setFrontDown(frontDown);
            intake.setRearDown(rearDown);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See FeederSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        double d_frontPower = frontPower.getAsDouble();
        double d_rearPower = rearPower.getAsDouble();
        if (d_frontPower == previousFrontPower && d_rearPower == previousRearPower) {
            return;
        }
        intake.setPower2(d_frontPower, d_rearPower);
        previousFrontPower = d_frontPower;
        previousRearPower = d_rearPower;
    }

    public void setFrontDown(boolean state) {
        frontDown = state;
    }

    public void setRearDown(boolean state) {
        rearDown = state;
    }

    public boolean getFrontDown() {
        return frontDown;
    }

    public boolean getRearDown() {
        return rearDown;
    }
}
