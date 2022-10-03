package frc.robot.commands.Autonomous.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.TurretPositionCommand;
import frc.robot.commands.AutomatedCommands.DoubleShotCommand;
import frc.robot.commands.AutomatedCommands.SeekCommand;
import frc.robot.commands.AutomatedCommands.StopLaunchCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {

    private DrivetrainSubsystem drivetrain;

    public TwoBallAuto(DrivetrainSubsystem drivetrain, IntakeSubsystem frontIntake, IntakeSubsystem rearIntake,
            ChimneySubsystem chimney,
            KickerSubsystem kicker, LauncherSubsystem launcher, LimelightSubsystem limelight,
            TurretSubsystem turret) {

        this.drivetrain = drivetrain;

        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        Trajectory trajectory3 = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath();
            trajectory1 = TrajectoryUtil
                    .fromPathweaverJson(trajectoryPath.resolve("output/Two Ball First.wpilib.json"));
            trajectory2 = TrajectoryUtil
                    .fromPathweaverJson(trajectoryPath.resolve("output/Screw Other Ball.wpilib.json"));
            trajectory3 = TrajectoryUtil
                    .fromPathweaverJson(trajectoryPath.resolve("output/Screw Other Ball again.wpilib.json"));

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
        }

        final Trajectory traj1f = trajectory1;

        // Run path following command, then stop at the end.
        addCommands(
                new InstantCommand(
                        () -> {
                            // Reset odometry to the starting pose of the trajectory.
                            drivetrain.resetOdometry(traj1f.getInitialPose());
                        }),
                new IntakeCommand(rearIntake, () -> -0.8, false).withTimeout(0.1),

                new ParallelDeadlineGroup( // Intake system activate and intake first ball
                        getRamseteCommand(trajectory1),
                        new PerpetualCommand(new SeekCommand(launcher, limelight, turret, false))),

                new DoubleShotCommand(chimney, turret, kicker, launcher, limelight),

                new StopLaunchCommand(launcher, kicker, chimney, turret),

                new ParallelDeadlineGroup( // Intake system activate and intake first ball
                        getRamseteCommand(trajectory2),
                        new IntakeCommand(frontIntake, () -> -0.8, true)),

                new ParallelDeadlineGroup( // Intake system activate and intake first ball
                        getRamseteCommand(trajectory3),
                        new IntakeCommand(rearIntake, () -> -0.8, false)),

                new ParallelRaceGroup(
                        new TurretPositionCommand(turret, limelight, 25000),
                        new LauncherCommand(launcher, () -> 6000, () -> false)),

                new SequentialCommandGroup(
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity(), () -> false)
                                .withTimeout(0.2),
                        new ChimneyCommand(chimney, () -> 0)
                                .withTimeout(0.1),
                        new KickerCommand(kicker, () -> 0.75)
                                .withTimeout(0.3),
                        new KickerCommand(kicker, () -> 0).withTimeout(0.05),
                        new ChimneyCommand(chimney, () -> 1)
                                .withTimeout(0.5),
                        new ChimneyCommand(chimney, () -> 0)
                                .withTimeout(0.15),
                        new KickerCommand(kicker, () -> 0.75)
                                .withTimeout(0.4)),

                new StopLaunchCommand(launcher, kicker, chimney, turret));

    }

    private Command getRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
    }
}
