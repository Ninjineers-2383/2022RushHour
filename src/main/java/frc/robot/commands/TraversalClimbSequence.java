package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.subsystems.ClimberSubsystemNew;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TraversalClimbSequence extends SequentialCommandGroup {
    private IntakeSubsystem intake;
    private ClimberSubsystemNew climber;
    private DrivetrainSubsystem drivetrain;
    private double climberPower = 0.75;

    public TraversalClimbSequence(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ClimberSubsystemNew climber) {

        this.intake = intake;
        this.climber = climber;
        this.drivetrain = drivetrain;

        addRequirements(intake, climber);

        drivetrain.toggleTippingDisabled();

        // Run path following command, then stop at the end.
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup( // hooooooking on mid bar
                                new AutoForward(drivetrain, 0.75, 1, 0.5, 2),
                                new IntakeCommand(intake, () -> 0, true, false),
                                new TraversalClimbAuto(climber, () -> climberPower, 250.0)),
                        new TraversalClimbAuto(climber, () -> climberPower, 690.0).withTimeout(3), // swing to high
                        new TraversalClimbAuto(climber, () -> -climberPower, 420.0).withTimeout(2), // unhook from mid
                        new TraversalClimbAuto(climber, () -> climberPower, 666.0).withTimeout(3), // swing to traversal
                        new TraversalClimbAuto(climber, () -> -climberPower, 999.0).withTimeout(2), // unhook from high
                        new TraversalClimbAuto(climber, () -> climberPower, 179.0).withTimeout(0.987) // swing robot
                                                                                                      // straight
                ));
    }
}
