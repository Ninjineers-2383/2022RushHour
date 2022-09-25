package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPositionCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TurretSubsystem turret;
    private final int position;
    private boolean done = false;

    public TurretPositionCommand(TurretSubsystem turret, int position) {
        this.turret = turret;
        this.position = position;

        addRequirements(turret);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        done = turret.runToPosition(position);
    }

    @Override
    public void end(boolean force) {
        turret.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
