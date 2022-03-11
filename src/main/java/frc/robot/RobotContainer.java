package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.FeedIn;
import frc.robot.commands.FeedOut;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.commands.Autonomous.AutoTurn;
import frc.robot.commands.Autonomous.LimelightCommandAuto;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    // Controllers and ports
    final XboxController driverController = new XboxController(0);
    final XboxController operatorController = new XboxController(1);

    // Operator Controls - Shooting, aiming, and climbing
    // Climber
    final JoystickButton climberUp = new JoystickButton(operatorController, Button.kY.value);
    final JoystickButton climberDown = new JoystickButton(operatorController, Button.kA.value);
    final JoystickButton climberInvert = new JoystickButton(operatorController, Button.kStart.value); // Right extra
                                                                                                      // button
    // Hook
    final JoystickButton hookUp = new JoystickButton(operatorController, Button.kX.value);
    final JoystickButton hookDown = new JoystickButton(operatorController, Button.kB.value);
    private DoubleSupplier climberPower = () -> climberUp.get() ? 1 : climberDown.get() ? -1 : 0;
    private DoubleSupplier hookPower = () -> hookUp.get() ? 0.7 : hookDown.get() ? -0.5 : -0.1;

    // Shooting
    final POVButton launchLowButton = new POVButton(operatorController, 180, 0);
    final JoystickButton indexerUp = new JoystickButton(operatorController, Button.kLeftBumper.value);
    final JoystickButton indexerUpTwoBall = new JoystickButton(operatorController, Button.kRightBumper.value);
    final POVButton indexerDown = new POVButton(operatorController, 0, 0);

    // Aiming
    final POVButton limelightTarget = new POVButton(operatorController, 270, 0);
    final POVButton limelightYeet = new POVButton(operatorController, 90, 0);

    // Driver Controls - Driving, feeding
    // Feeders
    final JoystickButton pooperPanicButton = new JoystickButton(driverController, Button.kBack.value); // left special
    final JoystickButton lowerFrontFeeder = new JoystickButton(driverController, Button.kRightBumper.value);
    final JoystickButton lowerBackFeeder = new JoystickButton(driverController, Button.kLeftBumper.value);
    final JoystickButton feedOut = new JoystickButton(driverController, Button.kA.value);
    public final IntakeSubsystem intake = new IntakeSubsystem();
    private DoubleSupplier intakePower = () -> feedOut.get() ? 1 : intake.getRearUp() && intake.getFrontUp() ? 0 : -1;

    // Driving
    private DoubleSupplier throttle = () -> driverController.getLeftY();
    private DoubleSupplier turn = () -> driverController.getRightX();

    // defining subsystems
    public final LimelightSubsystem limelight = new LimelightSubsystem();
    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public final ChimneySubsystem chimney = new ChimneySubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(intake, chimney);

    // defining premeditated commands
    private final LimelightCommand aimCommand = new LimelightCommand(limelight, () -> turret.getCurrentPosition(),
            () -> drivetrain.getAverageVelocity(), false);
    private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
    private final ClimberCommand climberCommand = new ClimberCommand(climber, climberPower, hookPower);

    // Custom Triggers
    public final FeedIn pooperIn = new FeedIn(colorSensor);
    public final FeedOut pooperOut = new FeedOut(colorSensor);

    // Auto Chooser
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putNumber("Launcher Velocity", 0.0);
        SmartDashboard.putNumber("Auto Duration", 0.0);

        // default commands for functions
        drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, throttle, turn));
        limelight.setDefaultCommand(aimCommand);
        turret.setDefaultCommand(new TurretCommand(turret, true, 6300));
        indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0));
        launcher.setDefaultCommand(
                new LauncherCommand(launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));
        chimney.setDefaultCommand(
                new ChimneyCommand(chimney, () -> colorSensor.getActive() ? -0.16 : intakePower.getAsDouble(), intake));
        intake.setDefaultCommand(intakeCommand);
        climber.setDefaultCommand(climberCommand);
        SmartDashboard.putBoolean("Aim Active", false);

        SetAutoCommands();
    }

    private void configureButtonBindings() {
        pooperIn.whenActive(colorSensor.loadIn(intake.getFrontUp(), !intake.getRearUp()));

        pooperOut.whenActive(colorSensor.loadOut(() -> !intake.getFrontUp()));

        /*
         * parallel command that runs:
         * turret aiming
         * launcher rev
         * launching a ball on target lock
         */
        limelightTarget.whileHeld(new ParallelCommandGroup(
                new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()),
                new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek()),
                new StartEndCommand(() -> SmartDashboard.putBoolean("Aim Active", true),
                        () -> SmartDashboard.putBoolean("Aim Active", false))));

        limelightYeet.whileHeld(new ParallelCommandGroup(
                new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity() + 4000),
                new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek())));

        launchLowButton.whileHeld(new LauncherCommand(launcher, () -> 6000));

        // backup kicker control if limelight fails
        indexerUp.whileHeld(new IndexerCommand(indexer, () -> 1));
        indexerDown.whileHeld(new IndexerCommand(indexer, () -> -1));

        indexerUpTwoBall.whenPressed(new SequentialCommandGroup(
                new ChimneyCommand(chimney, () -> -0.4, intake).withTimeout(0.1),
                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.2),
                new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
                new ChimneyCommand(chimney, () -> -0.5, intake).withTimeout(0.05),
                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.2)));

        lowerFrontFeeder.toggleWhenPressed(
                new StartEndCommand(
                        () -> intakeCommand.setFrontDown(false),
                        () -> intakeCommand.setFrontDown(true)));

        lowerBackFeeder.toggleWhenPressed(
                new StartEndCommand(
                        () -> intakeCommand.setRearDown(false),
                        () -> intakeCommand.setRearDown(true)));

        pooperPanicButton.toggleWhenPressed(
                new StartEndCommand(
                        () -> colorSensor.setActiveFalse(),
                        () -> colorSensor.setActiveTrue()));

        climberInvert.toggleWhenPressed(
                new StartEndCommand(
                        () -> climber.invertMotorPowers(),
                        () -> climber.unInvertMotorPower()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void SetAutoCommands() {

        LimelightCommandAuto autoLimelight = new LimelightCommandAuto(limelight, () -> turret.getCurrentPosition(),
                () -> drivetrain.getAverageVelocity(), false);
        LimelightCommandAuto autoLimelight2 = new LimelightCommandAuto(limelight, () -> turret.getCurrentPosition(),
                () -> drivetrain.getAverageVelocity(), false);
        LimelightCommandAuto autoLimelight3 = new LimelightCommandAuto(limelight, () -> turret.getCurrentPosition(),
                () -> drivetrain.getAverageVelocity(), false);

        Command fourBallAuto = new SequentialCommandGroup(
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new IntakeCommand(intake, () -> -0.8, false, true).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.88, 5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.6),
                        new ParallelRaceGroup(
                                autoLimelight,
                                new TurretCommand(turret, () -> autoLimelight.getTurretPower() * 0.65,
                                        () -> autoLimelight.getTurretSeek()).withTimeout(1.2)),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.1),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4),
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
                                new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.3),
                                new ChimneyCommand(chimney, () -> -0.5, intake).withTimeout(0.15),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4))),
                new ParallelCommandGroup( // Stop launch system
                        new TurretCommand(turret, () -> 0, () -> false).withTimeout(0.1),
                        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
                        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -0.8, intake).withTimeout(0.1),
                        new AutoTurn(drivetrain, 13, 8, -0.4, 5)), // drives back and intakes human player ball
                new AutoForward(drivetrain, 11, 2.5, 0.9, 5),
                new AutoTurn(drivetrain, 29, 10, 0.6, 5),
                new AutoForward(drivetrain, 1.3, 0.5, 0.6, 2),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new LauncherCommand(launcher, () -> 16500).withTimeout(0.1),
                        new AutoTurn(drivetrain, 24, 10, -0.6, 2)),
                new AutoForward(drivetrain, 9.5, 2, -0.88, 2),
                new ParallelRaceGroup(
                        autoLimelight2,
                        new TurretCommand(turret, () -> autoLimelight2.getTurretPower() * 0.65,
                                () -> autoLimelight2.getTurretSeek(), true).withTimeout(1.2),
                        new AutoForward(drivetrain, 4, 2, -0.88, 2)),
                new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek(), true)
                        .withTimeout(0.5),
                new ParallelCommandGroup( // Shoot two balls
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity() - 1000).withTimeout(2),
                        new SequentialCommandGroup(
                                new WaitCommand(0.4),
                                new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.1),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.3),
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                                new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.2),
                                new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.05),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.3))),
                new InstantCommand(colorSensor::setActiveTrue, colorSensor));

        Command twoBallAuto = new SequentialCommandGroup(
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new IntakeCommand(intake, () -> -1, false, true).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.9),
                        new ParallelRaceGroup(
                                autoLimelight3,
                                new TurretCommand(turret, () -> autoLimelight3.getTurretPower(),
                                        () -> autoLimelight3.getTurretSeek())
                                                .withTimeout(1.2)),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.1),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.5),
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
                                new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.3),
                                new ChimneyCommand(chimney, () -> -0.5, intake).withTimeout(0.15),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.5))),
                new ParallelCommandGroup( // Stop launch system
                        new TurretCommand(turret, () -> 0, () -> false).withTimeout(0.1),
                        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
                        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1)),
                new AutoTurn(drivetrain, 60, 10, 0.6, 6),
                new IntakeCommand(intake, () -> -1, true, false).withTimeout(0.1),
                new AutoForward(drivetrain, 5, 2, -0.6, 5),
                new TurretCommand(turret, true, -1000).withTimeout(0.6),

                new LauncherCommand(launcher, () -> 8000).withTimeout(1),
                new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4),
                new InstantCommand(colorSensor::setActiveTrue, colorSensor));

        Command nullAuto = null;

        Command oneBallAuto = new SequentialCommandGroup(
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.9),
                        new TurretCommand(turret, () -> aimCommand.getTurretPower() * 1.5,
                                () -> aimCommand.getTurretSeek())
                                        .withTimeout(1.2),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(2))),
                new InstantCommand(colorSensor::setActiveTrue, colorSensor));

        Command testAuto = new SequentialCommandGroup(
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.9),
                        new TurretCommand(turret, () -> aimCommand.getTurretPower() * 1.5,
                                () -> aimCommand.getTurretSeek())
                                        .withTimeout(1.2),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new IndexerCommand(indexer, () -> 0.75).withTimeout(2)),
                        new InstantCommand(colorSensor::setActiveTrue, colorSensor)));

        autoChooser.setDefaultOption("Two Ball", twoBallAuto);
        autoChooser.addOption("Four Ball", fourBallAuto);
        autoChooser.addOption("No Auto", nullAuto);
        autoChooser.addOption("One Ball", oneBallAuto);
        autoChooser.addOption("Test Auto", testAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
