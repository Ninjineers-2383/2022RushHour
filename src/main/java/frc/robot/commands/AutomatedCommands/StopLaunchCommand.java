package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class StopLaunchCommand extends SequentialCommandGroup {
    public StopLaunchCommand(LauncherSubsystem launcher, KickerSubsystem kicker, ChimneySubsystem chimney,
            TurretSubsystem turret) {
        addCommands(
                new LauncherCommand(launcher, () -> 0, () -> false).withTimeout(0.01),
                new KickerCommand(kicker, () -> 0.0).withTimeout(0.01),
                new ChimneyCommand(chimney, () -> 1).withTimeout(0.01));
    }
}
