package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPositionCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TurretSubsystem turret;
    private final LimelightSubsystem limelight;
    private final int position;
    private boolean done = false;

    public TurretPositionCommand(TurretSubsystem turret, LimelightSubsystem limelight, int position) {
        this.turret = turret;
        this.limelight = limelight;
        this.position = position;

        addRequirements(turret);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!limelight.getLockedOn() && !limelight.getTargetVisible()) {
            if (Math.abs(turret.getCurrentPosition() - position) > 300) {
                done = turret.runToPosition(position);
            }
        }
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
