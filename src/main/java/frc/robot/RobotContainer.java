// JA 4/13 11:30am changed:
// ClimberSubsytem to ClimberSubsystemNew
// ClimberCommand to ClimberCommandNew
// Added ClimberButtonAnalog

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Turret;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.ClimberCommandNew;
import frc.robot.commands.DoubleIntakeCommand;
import frc.robot.commands.DoubleShotCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.SeekCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoAlign;
import frc.robot.commands.Autonomous.AutoForwardAim;
import frc.robot.commands.Autonomous.autos.FiveBallAuto;
import frc.robot.commands.Autonomous.autos.FourBallAuto;
import frc.robot.commands.Autonomous.autos.OneBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAuto;
import frc.robot.commands.triggers.FeedIn;
import frc.robot.commands.triggers.FeedOut;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystemNew;
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
    final JoystickButton hookDown = new JoystickButton(operatorController, Button.kB.value);
    final JoystickButton autoAlign = new JoystickButton(driverController, Button.kRightStick.value);
    private DoubleSupplier climberPowerButton = () -> climberUp.get() ? .3 : climberDown.get() ? -.3 : 0;
    private DoubleSupplier climberPowerAnalog = () -> Math.abs(operatorController.getLeftY()) > .1
            ? operatorController.getLeftY()
            : 0;
    private DoubleSupplier hookPower = () -> hookUp.get() ? 0.7 : hookDown.get() ? -0.7 : -0.1;

    // Shooting
    // final POVButton launchLowButton = new POVButton(operatorController, 180, 0);
    final JoystickButton indexerUp = new JoystickButton(operatorController, Button.kLeftBumper.value);
    final JoystickButton indexerUpTwoBall = new JoystickButton(operatorController, Button.kRightBumper.value);
    final POVButton indexerDown = new POVButton(operatorController, 0, 0);

    // Aiming
    final POVButton limelightTarget = new POVButton(operatorController, 270, 0);
    // final POVButton limelightYeet = new POVButton(operatorController, 90, 0);
    final POVButton fixedHighGoal = new POVButton(operatorController, 180, 0);
    final POVButton testDoubleShot = new POVButton(operatorController, 90, 0);

    // Driver Controls - Driving, feeding
    // Feeders
    // final JoystickButton pooperPanicButton = new JoystickButton(driverController,
    // Button.kBack.value); // left
    // special
    final JoystickButton lowerFrontFeeder = new JoystickButton(driverController, Button.kRightBumper.value);
    final JoystickButton lowerBackFeeder = new JoystickButton(driverController, Button.kLeftBumper.value);
    final JoystickButton feedOut = new JoystickButton(driverController, Button.kA.value);
    final JoystickButton chimneyUp = new JoystickButton(driverController, Button.kY.value);
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
    public final ClimberSubsystemNew climber = new ClimberSubsystemNew();
    public final ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(intake, chimney); // was intake
                                                                                               // and
    // chimney subsystems
    public final CameraSubsystem rearCamera = new CameraSubsystem("rear");

    // defining premeditated commands
    private final LimelightCommand aimCommand = new LimelightCommand(limelight, () -> turret.getCurrentPosition(),
            drivetrain.getAverageVelocity());
    private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
    private final ClimberCommandNew climberCommandNew = new ClimberCommandNew(climber, climberPowerAnalog, hookPower);
    private Trigger driverFrontFeed = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);
    private Trigger driverBackFeed = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

    // Custom Triggers
    public final FeedIn pooperIn = new FeedIn(colorSensor);
    public final FeedOut pooperOut = new FeedOut(colorSensor);
    public final Trigger autoShoot = new Trigger(
            () -> aimCommand.getLockedOn() && limelight.getTargetVisible() && launcher.isReady()
                    && Math.abs(drivetrain.getAverageVelocity().getAsDouble()) < 0.1);

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
                        intakePower));
        intake.setDefaultCommand(intakeCommand);
        climber.setDefaultCommand(climberCommandNew);
        SmartDashboard.putBoolean("Aim Active", false);

        SetAutoCommands();
    }

    private void configureButtonBindings() {

        autoAlign.whileHeld(new AutoAlign(drivetrain, rearCamera, throttle, turn));

        pooperIn.whileActiveContinuous(colorSensor.loadIn(intake.getFrontUp(),
                !intake.getRearUp()), false);

        pooperOut.whileActiveContinuous(colorSensor.loadOut(() -> (!intake.getFrontUp())), false);

        feedOut.whileHeld(new ParallelCommandGroup(
                new DoubleIntakeCommand(intake, () -> 1, () -> -1),
                new ChimneyCommand(chimney, () -> 1)));

        fixedHighGoal.whileHeld(new LauncherCommand(launcher, () -> 14000));

        // backup kicker control if limelight fails
        indexerUp.whileHeld(new IndexerCommand(indexer, () -> 1));

        indexerDown.whileHeld(new IndexerCommand(indexer, () -> -1));

        indexerUpTwoBall.and(autoShoot.negate())
                .whileActiveContinuous(new SeekCommand(launcher, limelight, turret, aimCommand));

        indexerUpTwoBall.and(autoShoot).whenActive(
                new DoubleShotCommand(chimney, turret, aimCommand, indexer, launcher, limelight),
                false);

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

    private Command getTestAuto() {
        return new SequentialCommandGroup(
                new TurretCommand(turret, Turret.OFFSET_TICKS).withTimeout(0.75),
                new AutoForwardAim(drivetrain, rearCamera, 11.8, 2.3, 0.4, 50));
    }

    private void SetAutoCommands() {
        Command oneBallAuto = new OneBallAuto(drivetrain, indexer, launcher, limelight, turret, aimCommand);
        Command twoBallAuto = new TwoBallAuto(drivetrain, intake, chimney, indexer, launcher, limelight, turret,
                aimCommand);
        Command fourBallAuto = new FourBallAuto(drivetrain, intake, chimney, indexer, launcher, limelight, turret,
                aimCommand);
        Command fiveBallAuto = new FiveBallAuto(drivetrain, intake, chimney, indexer, launcher, limelight, turret,
                aimCommand);

        Command nullAuto = null;

        Command testAuto = getTestAuto();

        autoChooser.addOption("One Ball", oneBallAuto);
        autoChooser.setDefaultOption("Two Ball", twoBallAuto);
        autoChooser.addOption("Four Ball", fourBallAuto);
        autoChooser.addOption("Five Ball", fiveBallAuto);

        autoChooser.addOption("Test Auto", testAuto);
        autoChooser.addOption("No Auto", nullAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
