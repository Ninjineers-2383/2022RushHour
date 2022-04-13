package frc.robot.commands.Autonomous.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Turret;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.commands.Autonomous.AutoTurn;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FourBallAuto extends SequentialCommandGroup {

    public FourBallAuto(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ChimneySubsystem chimney,
            IndexerSubsystem indexer, LauncherSubsystem launcher, LimelightSubsystem limelight, TurretSubsystem turret,
            LimelightCommand aimCommand) {
        // Run path following command, then stop at the end.
        addCommands(
                new TurretCommand(turret, Turret.OFFSET_TICKS).withTimeout(0.5),
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new ChimneyCommand(chimney, () -> -0.75).withTimeout(0.1),
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new IntakeCommand(intake, () -> -0.8, false, true).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.88, 10)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new ParallelRaceGroup(
                                new TurretCommand(turret,
                                        () -> aimCommand.getTurretPower(),
                                        () -> aimCommand.getTurretSeek())
                                        .withTimeout(1.5)),
                        new SequentialCommandGroup(
                                new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity())
                                        .withTimeout(0.2),
                                new ChimneyCommand(chimney, () -> 0)
                                        .withTimeout(0.1),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(0.3),
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
                                new ChimneyCommand(chimney, () -> -1)
                                        .withTimeout(0.5),
                                new ChimneyCommand(chimney, () -> -0.5)
                                        .withTimeout(0.15),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(0.4))),
                new ParallelCommandGroup( // Stop launch system
                        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
                        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -0.8).withTimeout(0.1),
                        new TurretCommand(turret, Turret.OFFSET_TICKS).withTimeout(0.5),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                // drives back and intakes
                                new AutoTurn(drivetrain, 13.5, 8, -0.4, 5))),
                // human player ball
                new AutoForward(drivetrain, 11.6, 2.5, 0.9, 5).withTimeout(5),
                new WaitCommand(1.5),
                new ParallelRaceGroup(
                        new LauncherCommand(launcher, () -> 16500),
                        new TurretCommand(turret,
                                -18000),
                        new AutoForward(drivetrain, 11.5, 1.7, -0.88, 20)),
                new ParallelRaceGroup(
                        new TurretCommand(turret, () -> aimCommand.getTurretPower(),
                                () -> aimCommand.getTurretSeek(), true)
                                .withTimeout(1),
                        new LauncherCommand(launcher,
                                () -> limelight.getLaunchingVelocity())),
                new TurretCommand(turret, () -> aimCommand.getTurretPower(),
                        () -> aimCommand.getTurretSeek(), true)
                        .withTimeout(0.2),
                new ChimneyCommand(chimney, () -> 0)
                        .withTimeout(0.1),
                new IndexerCommand(indexer, () -> 0.75)
                        .withTimeout(0.3),
                new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                new ChimneyCommand(chimney, () -> -1)
                        .withTimeout(0.2),
                new ChimneyCommand(chimney, () -> 0)
                        .withTimeout(0.05),
                new IndexerCommand(indexer, () -> 0.75)
                        .withTimeout(0.5));
    }
}
