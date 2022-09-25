package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.TurretSeekCommand;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SeekCommand extends ParallelCommandGroup {
    private final LimelightSubsystem limelight;
    private final LauncherSubsystem launcher;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public SeekCommand(LauncherSubsystem launcher, LimelightSubsystem limelight, TurretSubsystem turret,
            boolean seekDirection) {
        this.limelight = limelight;
        this.launcher = launcher;
        addCommands(
                new LauncherCommand(launcher,
                        () -> (limelight
                                .getLaunchingVelocity()),
                        () -> true).perpetually(),
                new TurretSeekCommand(turret, () -> limelight.getTurretPower(),
                        () -> limelight.getTurretSeek(),
                        seekDirection),
                new StartEndCommand(
                        () -> SmartDashboard.putBoolean("Aim Active", true),
                        () -> SmartDashboard.putBoolean("Aim Active", false)));
    }

    @Override
    public boolean isFinished() {
        return limelight.getLockedOn() && limelight.getTargetVisible() && launcher.isReady();
    }
}
