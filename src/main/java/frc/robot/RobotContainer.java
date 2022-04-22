// JA 4/13 11:30am changed:
// ClimberSubsytem to ClimberSubsystemNew
// ClimberCommand to ClimberCommandNew
// Added ClimberButtonAnalog

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Turret;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.DoubleIntakeCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TraversalClimbManualCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.AutomatedCommands.ClimbSetPosition;
import frc.robot.commands.AutomatedCommands.DoubleShotCommand;
import frc.robot.commands.AutomatedCommands.SeekCommand;
import frc.robot.commands.Autonomous.autos.FiveBallAuto;
import frc.robot.commands.Autonomous.autos.FourBallAuto;
import frc.robot.commands.Autonomous.autos.OneBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAutoSimple;
import frc.robot.commands.triggers.AutoShoot;
import frc.robot.commands.triggers.FeedIn;
import frc.robot.commands.triggers.FeedOut;
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

    // Auto Chooser
    SendableChooser<Command> autoChooser = new SendableChooser<>();
    SendableChooser<Integer> climbChooser = new SendableChooser<>();

    // Controllers and ports
    final XboxController driverController = new XboxController(0);
    final XboxController operatorController = new XboxController(1);

    // Operator Controls - Shooting, aiming, and climbing
    // Climber
    final JoystickButton climberInvert = new JoystickButton(operatorController, Button.kStart.value); // Right extra
                                                                                                      // button
    final JoystickButton tippingToggle = new JoystickButton(operatorController, Button.kBack.value);

    // Climber Hook
    final JoystickButton autoAlign = new JoystickButton(driverController, Button.kRightStick.value);
    // private DoubleSupplier climberPowerButton = () -> climberUp.get() ? .3 :
    // climberDown.get() ? -.3 : 0;
    private DoubleSupplier climberPowerAnalog = () -> Math.abs(operatorController.getLeftY()) > .1
            ? operatorController.getLeftY()
            : 0;

    // Shooting
    // final POVButton launchLowButton = new POVButton(operatorController, 180, 0);
    final JoystickButton indexerUp = new JoystickButton(operatorController, Button.kY.value);
    final JoystickButton indexerDown = new JoystickButton(operatorController, Button.kX.value);
    final JoystickButton indexerUpTwoBall = new JoystickButton(operatorController, Button.kRightBumper.value);
    final JoystickButton indexerOverride = new JoystickButton(operatorController, Button.kLeftBumper.value);

    // Aiming
    final POVButton limelightTarget = new POVButton(operatorController, 270, 0);
    // final POVButton limelightYeet = new POVButton(operatorController, 90, 0);
    final POVButton fixedHighGoal = new POVButton(operatorController, 180, 0);

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
    final JoystickButton climbToPosition = new JoystickButton(operatorController, Button.kBack.value);
    final JoystickButton climbSequence = new JoystickButton(operatorController, Button.kA.value);
    final JoystickButton climbCancel = new JoystickButton(operatorController, Button.kB.value);

    public final IntakeSubsystem intake = new IntakeSubsystem();
    private DoubleSupplier intakePower = () -> driverController.getLeftTriggerAxis() * -1
            + driverController.getRightTriggerAxis() * -1;

    // Driving
    private DoubleSupplier throttle = () -> driverController.getLeftY();
    private DoubleSupplier turn = () -> driverController.getRightX() * 0.8;
    private double climbPosition = 0;

    // defining subsystems
    public final LimelightSubsystem limelight = new LimelightSubsystem();
    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public final ChimneySubsystem chimney = new ChimneySubsystem();
    public final ClimberSubsystemNew climber = new ClimberSubsystemNew();
    public final ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(intake, chimney);

    // defining premeditated commands
    private final LimelightCommand aimCommand = new LimelightCommand(limelight, () -> turret.getCurrentPosition(),
            drivetrain.getAverageVelocity());
    private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);

    private final SeekCommand seekCommand = new SeekCommand(launcher, limelight, turret, aimCommand, false);
    private Trigger driverFrontFeed = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);
    private Trigger driverBackFeed = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

    // Custom Triggers
    public final FeedIn pooperIn = new FeedIn(colorSensor);
    public final FeedOut pooperOut = new FeedOut(colorSensor);
    public final Trigger autoShoot = new AutoShoot(() -> aimCommand.getLockedOn(), () -> launcher.isReady(),
            drivetrain.getAverageVelocity());

    private final TraversalClimbManualCommand traversalManual = new TraversalClimbManualCommand(climber,
            climberPowerAnalog);
    private final ClimbSetPosition traversalAuto = new ClimbSetPosition(climber, () -> climbChooser.getSelected());
    private Command climbFULLLAUTOOOOO;

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putNumber("Launcher Velocity", 0.0);
        SmartDashboard.putNumber("Auto Duration", 0.0);
        SmartDashboard.putBoolean("Auto Climb", false);

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
        climber.setDefaultCommand(traversalManual);
        SmartDashboard.putBoolean("Aim Active", false);
        // SmartDashboard.putBoolean("Climb Button", false);

        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5800, "10.23.83.11", 5800);

        SetAutoCommands();
    }

    private void configureButtonBindings() {

        pooperIn.whileActiveContinuous(colorSensor.loadIn(intake.getFrontUp(),
                !intake.getRearUp()), false);

        pooperOut.whileActiveContinuous(colorSensor.loadOut(() -> (!intake.getFrontUp())), false);

        feedOut.whileHeld(new ParallelCommandGroup(
                new DoubleIntakeCommand(intake, () -> 1, () -> -1),
                new ChimneyCommand(chimney, () -> 1)));

        fixedHighGoal.whileHeld(new LauncherCommand(launcher, () -> 16000));

        // backup kicker control if limelight fails
        indexerUp.whileHeld(new ParallelCommandGroup(
                new IndexerCommand(indexer, () -> 1),
                new ChimneyCommand(chimney, () -> -1)));

        indexerDown.whileHeld(new IndexerCommand(indexer, () -> -1));

        indexerUpTwoBall.and(autoShoot.negate())
                .whileActiveContinuous(seekCommand);

        indexerUpTwoBall.and(autoShoot.or(
                indexerOverride)).whenActive(
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

        climbChooser.addOption("Zero", 0);
        climbChooser.addOption("Align", 70); // Previously 75
        climbChooser.addOption("High", 210);
        climbChooser.addOption("Mid unlatch", 140);
        climbChooser.addOption("Traverse", 360);
        climbChooser.addOption("High unlatch", 295);

        SmartDashboard.putData("Climb Position", climbChooser);

        climbToPosition.whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            SmartDashboard.putBoolean("Auto Climb", true);
                            drivetrain.toggleTippingDisabled();
                            intakeCommand.setFrontDown(true);
                        }),
                        new ClimbSetPosition(climber, () -> climbChooser.getSelected()),
                        new InstantCommand(() -> SmartDashboard.putBoolean("Auto Climb",
                                false))));
        climberInvert.whenPressed(
                () -> climber.invertMotorPowers());

        barfToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> colorSensor.setActiveFalse(),
                        () -> colorSensor.setActiveTrue()));

        tippingToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> drivetrain.toggleTippingEnabled(),
                        () -> drivetrain.toggleTippingDisabled()));

        climbFULLLAUTOOOOO = new SequentialCommandGroup(
                new ClimbSetPosition(climber, () -> 210),
                new WaitCommand(0.5),
                new ClimbSetPosition(climber, () -> 140),
                new WaitCommand(0.5),
                new ClimbSetPosition(climber, () -> 360),
                new WaitCommand(0.5),
                new ClimbSetPosition(climber, () -> 295),
                new ClimbSetPosition(climber, () -> 360));
        climbSequence.whenActive(climbFULLLAUTOOOOO);
        climbCancel.whenActive(() -> climbFULLLAUTOOOOO.cancel());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void robotPeriodic() {
        // boolean move = SmartDashboard.getBoolean("Climb Button", false);
        // if (move) {
        // climber.setDefaultCommand(traversalAuto);
        // } else {
        // climber.setDefaultCommand(traversalManual);
        // }
    }

    private Command getTestAuto() {
        return null;
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

        Command simpleTwoBallAuto = new TwoBallAutoSimple(drivetrain, intake, chimney, indexer, launcher, limelight,
                turret,
                aimCommand);

        autoChooser.addOption("One Ball", oneBallAuto);
        autoChooser.addOption("Two Ball", twoBallAuto);
        autoChooser.addOption("Four Ball", fourBallAuto);
        autoChooser.setDefaultOption("Five Ball", fiveBallAuto);
        autoChooser.addOption("Two Ball Simple", simpleTwoBallAuto);

        autoChooser.addOption("Test Auto", testAuto);
        autoChooser.addOption("No Auto", nullAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
