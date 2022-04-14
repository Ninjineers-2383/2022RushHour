package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TraversalClimbAutoCommand;
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

        addRequirements(intake, climber, drivetrain);

        drivetrain.toggleTippingDisabled();

        // Run path following command, then stop at the end.
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup( // hooooooking on mid bar
                                new AutoForward(drivetrain, 0.75, 1, 0.5, 2),
                                new IntakeCommand(intake, () -> 0, true, false),
                                new TraversalClimbAutoCommand(climber, () -> climberPower, 250.0)),
                        new TraversalClimbAutoCommand(climber, () -> climberPower, 690.0).withTimeout(2), // swing to
                                                                                                          // high
                        new TraversalClimbAutoCommand(climber, () -> -climberPower, 420.0).withTimeout(3), // unhook
                                                                                                           // from mid
                        new TraversalClimbAutoCommand(climber, () -> climberPower, 666.0).withTimeout(8), // swing to
                                                                                                          // traversal
                        new TraversalClimbAutoCommand(climber, () -> -climberPower, 999.0).withTimeout(3), // unhook
                                                                                                           // from high
                        new TraversalClimbAutoCommand(climber, () -> climberPower, 179.0).withTimeout(0.987) // swing
                                                                                                             // robot
                // straight
                ));
    }
}
