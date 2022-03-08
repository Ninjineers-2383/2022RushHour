package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class DoubleIntakeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final IntakeSubsystem intake;
    private DoubleSupplier frontPower;
    private DoubleSupplier rearPower;
    private boolean frontDown;
    private boolean rearDown;


    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a feeder command that takes in the feeder subsystem and runs feeder subsystem actions.
    public DoubleIntakeCommand(IntakeSubsystem intake, DoubleSupplier frontPower, DoubleSupplier rearPower, boolean frontDown, boolean rearDown) {
        this.intake = intake;
        this.frontPower = frontPower;
        this.rearPower = rearPower;
        this.frontDown = frontDown;
        this.rearDown = rearDown;
        addRequirements(intake);
    }


    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See FeederSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        intake.setPower2(frontPower.getAsDouble(), rearPower.getAsDouble());
        //m_subsystem.kickV(m_speed.getAsDouble());
        intake.setFrontDown(frontDown);
        intake.setRearDown(rearDown);
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
