package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class StopLaunchCommand extends SequentialCommandGroup {
    public StopLaunchCommand(LauncherSubsystem launcher, IndexerSubsystem indexer, ChimneySubsystem chimney,
            TurretSubsystem turret) {
        addCommands(
                new LauncherCommand(launcher, () -> 0).withTimeout(0.01),
                new IndexerCommand(indexer, () -> 0.0).withTimeout(0.01),
                new ChimneyCommand(chimney, () -> -0.8).withTimeout(0.01));

    }
}
