package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SeekCommand extends ParallelCommandGroup {
    private final LimelightCommand aimCommand;
    private final LimelightSubsystem limelight;
    private final LauncherSubsystem launcher;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public SeekCommand(LauncherSubsystem launcher, LimelightSubsystem limelight, TurretSubsystem turret,
            LimelightCommand aimCommand) {
        this.aimCommand = aimCommand;
        this.limelight = limelight;
        this.launcher = launcher;
        addCommands(
                new LauncherCommand(launcher,
                        () -> (limelight
                                .getLaunchingVelocity()),
                        () -> true),
                new TurretCommand(turret, () -> aimCommand.getTurretPower(),
                        () -> aimCommand.getTurretSeek(),
                        false),
                new StartEndCommand(
                        () -> SmartDashboard.putBoolean("Aim Active", true),
                        () -> SmartDashboard.putBoolean("Aim Active", false)));
    }

    @Override
    public boolean isFinished() {
        return aimCommand.getLockedOn() && limelight.getTargetVisible() && launcher.isReady();
    }
}
