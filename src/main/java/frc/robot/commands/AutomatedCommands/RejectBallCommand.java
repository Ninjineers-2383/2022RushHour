package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.TurretSeekCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RejectBallCommand extends SequentialCommandGroup {

    public RejectBallCommand(ChimneySubsystem chimney, TurretSubsystem turret,
            KickerSubsystem kicker, LauncherSubsystem launcher, LimelightSubsystem limelight) {
        addCommands(
                new ParallelDeadlineGroup(new KickerCommand(kicker, () -> 0.8).withTimeout(0.8),
                        new ChimneyCommand(chimney, () -> 1).perpetually(),
                        new TurretSeekCommand(turret, () -> 0,
                                () -> true, false),
                        new LauncherCommand(launcher,
                                () -> 5000,
                                () -> true)));

    }
}
