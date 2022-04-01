package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Turret;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DoubleIntakeCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.FeedIn;
import frc.robot.commands.FeedOut;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoAlign;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.commands.Autonomous.AutoForwardAim;
import frc.robot.commands.Autonomous.LimelightCommandAuto;
import frc.robot.subsystems.CameraSubsystem;
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
    final JoystickButton tippingToggle = new JoystickButton(operatorController, Button.kBack.value);

    // Climber Hook
    final JoystickButton hookUp = new JoystickButton(operatorController, Button.kX.value);
    final JoystickButton autoAlign = new JoystickButton(driverController, Button.kRightStick.value);
    private DoubleSupplier climberPower = () -> climberUp.get() ? 1 : climberDown.get() ? -1 : 0;
    private DoubleSupplier hookPower = () -> hookUp.get() ? 0.7 : -0.1;

    // Shooting
    final POVButton launchLowButton = new POVButton(operatorController, 180, 0);
    final JoystickButton indexerUp = new JoystickButton(operatorController, Button.kLeftBumper.value);
    final JoystickButton indexerUpTwoBall = new JoystickButton(operatorController, Button.kRightBumper.value);
    final POVButton indexerDown = new POVButton(operatorController, 0, 0);

    // Aiming
    final POVButton limelightTarget = new POVButton(operatorController, 270, 0);
    // final POVButton limelightYeet = new POVButton(operatorController, 90, 0);
    final POVButton fixedHighGoal = new POVButton(operatorController, 90, 0);

    // Driver Controls - Driving, feeding
    // Feeders
    // final JoystickButton pooperPanicButton = new JoystickButton(driverController,
    // Button.kBack.value); // left
    // special
    final JoystickButton lowerFrontFeeder = new JoystickButton(driverController, Button.kRightBumper.value);
    final JoystickButton lowerBackFeeder = new JoystickButton(driverController, Button.kLeftBumper.value);
    final JoystickButton feedOut = new JoystickButton(driverController, Button.kA.value);
    final JoystickButton chimUp = new JoystickButton(driverController, Button.kY.value);
    final JoystickButton barfToggle = new JoystickButton(operatorController, Button.kStart.value);
    public final IntakeSubsystem intake = new IntakeSubsystem();
    private DoubleSupplier intakePower = () -> driverController.getLeftTriggerAxis() * -1
            + driverController.getRightTriggerAxis() * -1;

    // Driving
    private DoubleSupplier throttle = () -> driverController.getLeftY();
    private DoubleSupplier turn = () -> driverController.getRightX() * 0.8;

    // defining subsystems
    public final LimelightSubsystem limelight = new LimelightSubsystem();

    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public final ChimneySubsystem chimney = new ChimneySubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(intake, chimney); // was intake
                                                                                               // and
    // chimney subsystems
    public final CameraSubsystem rearCamera = new CameraSubsystem("rear");

    // defining premeditated commands
    private final LimelightCommand aimCommand = new LimelightCommand(limelight, () -> turret.getCurrentPosition(),
            drivetrain.getAverageVelocity());
    private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
    private final ClimberCommand climberCommand = new ClimberCommand(climber, climberPower, hookPower);
    private Trigger driverFrontFeed = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);
    private Trigger driverBackFeed = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

    // Custom Triggers
    public final FeedIn pooperIn = new FeedIn(colorSensor);
    public final FeedOut pooperOut = new FeedOut(colorSensor);
    // public final FeedOut barfOut = new FeedOut(colorSensor);

    // Required robot state
    private boolean shouldAdjustTurret = false;

    private void setIsShooting() {
        shouldAdjustTurret = false;
    }

    private void setNotIsShooting() {
        shouldAdjustTurret = true;
    }

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
        turret.setDefaultCommand(new TurretCommand(turret, Turret.OFFSET_TICKS));
        indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0));
        launcher.setDefaultCommand(
                new LauncherCommand(launcher,
                        () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));
        chimney.setDefaultCommand(
                new ChimneyCommand(chimney,
                        () -> ((barfToggle.get() ? 0 : intakePower.getAsDouble()))));
        intake.setDefaultCommand(intakeCommand);
        climber.setDefaultCommand(climberCommand);
        SmartDashboard.putBoolean("Aim Active", false);

        SetAutoCommands();
    }

    private void configureButtonBindings() {

        autoAlign.whileHeld(new AutoAlign(drivetrain, rearCamera, throttle, turn));

        pooperIn.whileActiveContinuous(colorSensor.loadIn(intake.getFrontUp(),
                !intake.getRearUp()), false);

        pooperOut.whileActiveContinuous(colorSensor.loadOut(() -> (!intake.getFrontUp())), false);

        // barfOut.whenActive(colorSensor.shootOut());

        feedOut.whileHeld(new ParallelCommandGroup(
                new DoubleIntakeCommand(intake, () -> 1, () -> -1),
                new ChimneyCommand(chimney, () -> 1)));

        /*
         * parallel command that runs:
         * turret aiming
         * launcher rev
         * launching a ball on target lock
         */
        limelightTarget.whileHeld(new ParallelCommandGroup(
                new LauncherCommand(launcher,
                        () -> (limelight
                                .getLaunchingVelocity()),
                        () -> shouldAdjustTurret),
                new TurretCommand(turret, () -> aimCommand.getTurretPower(),
                        () -> aimCommand.getTurretSeek(),
                        () -> shouldAdjustTurret),
                new StartEndCommand(
                        () -> SmartDashboard.putBoolean("Aim Active", true),
                        () -> SmartDashboard.putBoolean("Aim Active", false))));

        // limelightYeet.whileHeld(new ParallelCommandGroup(
        // new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity() + 4000),
        // new TurretCommand(turret, () -> aimCommand.getTurretPower(),
        // () -> aimCommand.getTurretSeek())));

        fixedHighGoal.whileHeld(new LauncherCommand(launcher, () -> 13000));

        launchLowButton.whileHeld(new LauncherCommand(launcher, () -> 6000));

        // backup kicker control if limelight fails
        indexerUp.whileHeld(new ParallelCommandGroup(new IndexerCommand(indexer, () -> 1),
                new InstantCommand(() -> setNotIsShooting()))).whenInactive(new InstantCommand(
                        () -> setNotIsShooting()));

        indexerDown.whileHeld(new IndexerCommand(indexer, () -> -1));

        indexerUpTwoBall.whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            // setIsShooting();
                            driverController.setRumble(RumbleType.kLeftRumble, 1.0);
                            driverController.setRumble(RumbleType.kRightRumble, 1.0);
                            operatorController.setRumble(RumbleType.kLeftRumble, 1.0);
                            operatorController.setRumble(RumbleType.kRightRumble, 1.0);
                        }),
                        new ChimneyCommand(chimney, () -> -0.4).withTimeout(0.1),
                        new IndexerCommand(indexer, () -> 0.75).withTimeout(0.2),
                        new ParallelCommandGroup(
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                                new ChimneyCommand(chimney, () -> -1).withTimeout(0.22)),
                        new ChimneyCommand(chimney, () -> -0.5).withTimeout(0.05),
                        new IndexerCommand(indexer, () -> 0.75).withTimeout(0.2),
                        new InstantCommand(() -> {
                            driverController.setRumble(RumbleType.kLeftRumble, 0);
                            driverController.setRumble(RumbleType.kRightRumble, 0);
                            operatorController.setRumble(RumbleType.kLeftRumble, 0);
                            operatorController.setRumble(RumbleType.kRightRumble, 0);
                            // setNotIsShooting();
                        })));

        // lowerFrontFeeder.toggleWhenPressed(
        // new StartEndCommand(
        // () -> intakeCommand.setFrontDown(false),
        // () -> intakeCommand.setFrontDown(true)));

        // lowerBackFeeder.toggleWhenPressed(
        // new StartEndCommand(
        // () -> intakeCommand.setRearDown(false),
        // () -> intakeCommand.setRearDown(true)));

        driverFrontFeed.whenActive(
                new InstantCommand(
                        () -> intakeCommand.setFrontDown(true)));

        driverFrontFeed.whenInactive(
                new InstantCommand(
                        () -> intakeCommand.setFrontDown(false)));

        driverBackFeed.whenActive(
                new InstantCommand(
                        () -> intakeCommand.setRearDown(true)));

        driverBackFeed.whenInactive(
                new InstantCommand(
                        () -> intakeCommand.setRearDown(false)));

        lowerFrontFeeder.toggleWhenPressed(
                new StartEndCommand(
                        () -> intakeCommand.setFrontDown(false),
                        () -> intakeCommand.setFrontDown(true)));

        lowerBackFeeder.toggleWhenPressed(
                new StartEndCommand(
                        () -> intakeCommand.setRearDown(false),
                        () -> intakeCommand.setRearDown(true)));

        // pooperPanicButton.toggleWhenPressed(
        // new StartEndCommand(
        // () -> colorSensor.setActiveFalse(),
        // () -> colorSensor.setActiveTrue()));

        climberInvert.toggleWhenPressed(
                new StartEndCommand(
                        () -> climber.invertMotorPowers(),
                        () -> climber.unInvertMotorPower()));

        barfToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> colorSensor.setActiveFalse(),
                        () -> colorSensor.setActiveTrue()));

        tippingToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> drivetrain.toggleTippingEnabled(),
                        () -> drivetrain.toggleTippingDisabled()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void SetAutoCommands() {

        LimelightCommandAuto autoLimelight = new LimelightCommandAuto(limelight,
                () -> turret.getCurrentPosition(),
                drivetrain.getAverageVelocity(), false);
        LimelightCommandAuto autoLimelight2 = new LimelightCommandAuto(limelight,
                () -> turret.getCurrentPosition(),
                drivetrain.getAverageVelocity(), false);
        LimelightCommandAuto autoLimelight3 = new LimelightCommandAuto(limelight,
                () -> turret.getCurrentPosition(),
                drivetrain.getAverageVelocity(), false);

        Command fourBallAuto = new SequentialCommandGroup(
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
                                new AutoAlign(drivetrain, rearCamera, 0.5).withTimeout(1) // drives back and intakes
                        )),
                // human player ball
                new AutoForwardAim(drivetrain, rearCamera, 11.8, 2.3, 0.8, 50),
                new WaitCommand(0.4),
                new ParallelRaceGroup(
                        new LauncherCommand(launcher, () -> 16500),
                        new TurretCommand(turret,
                                -18000),
                        new AutoForward(drivetrain, 13, 1.7, -0.88, 20)),
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

        Command twoBallAuto = new SequentialCommandGroup(
            new TurretCommand(turret, Turret.OFFSET_TICKS).withTimeout(0.5),
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new IntakeCommand(intake, () -> -1, false, true).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -1).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity())
                                .withTimeout(0.9),
                        new ParallelRaceGroup(
                                new TurretCommand(turret,
                                        () -> aimCommand.getTurretPower(),
                                        () -> aimCommand.getTurretSeek())
                                                .withTimeout(1.2)),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new ChimneyCommand(chimney, () -> 0)
                                        .withTimeout(0.1),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(0.5),
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
                                new ChimneyCommand(chimney, () -> -1)
                                        .withTimeout(0.3),
                                new ChimneyCommand(chimney, () -> -0.5)
                                        .withTimeout(0.15),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(0.5))),
                new ParallelCommandGroup( // Stop launch system
                        new TurretCommand(turret, () -> 0, () -> false).withTimeout(0.1),
                        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
                        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -1).withTimeout(0.1)));
        // new AutoTurn(drivetrain, 60, 10, 0.6, 6),
        // new IntakeCommand(intake, () -> -1, true, false).withTimeout(0.1),
        // new AutoForward(drivetrain, 5, 2, -0.6, 5),
        // new TurretCommand(turret, true, -1000).withTimeout(0.6),

        // new LauncherCommand(launcher, () -> 8000).withTimeout(1),
        // new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4),
        // new InstantCommand(colorSensor::setActiveTrue, colorSensor));

        Command nullAuto = null;

        Command oneBallAuto = new SequentialCommandGroup(
            new TurretCommand(turret, Turret.OFFSET_TICKS).withTimeout(0.5),
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity())
                                .withTimeout(0.9),
                        new TurretCommand(turret, () -> aimCommand.getTurretPower() * 1.5,
                                () -> aimCommand.getTurretSeek())
                                        .withTimeout(1.2),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(2))),
                new InstantCommand(colorSensor::setActiveTrue, colorSensor));

        Command testAuto = new TurretCommand(turret,
                () -> aimCommand.getTurretPower(),
                () -> aimCommand.getTurretSeek())
                        .withTimeout(3);

        autoChooser.setDefaultOption("Two Ball", twoBallAuto);
        autoChooser.addOption("Four Ball", fourBallAuto);
        autoChooser.addOption("No Auto", nullAuto);
        autoChooser.addOption("One Ball", oneBallAuto);
        autoChooser.addOption("Test Auto", testAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
