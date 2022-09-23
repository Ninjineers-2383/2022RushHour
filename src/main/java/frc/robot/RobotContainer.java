// JA 4/13 11:30am changed:
// ClimberSubsystem to ClimberSubsystemNew
// ClimberCommand to ClimberCommandNew
// Added ClimberButtonAnalog

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.mashape.unirest.http.HttpResponse;
import com.mashape.unirest.http.Unirest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Turret;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TraversalClimbManualCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.AutomatedCommands.DoubleShotCommand;
import frc.robot.commands.AutomatedCommands.SeekCommand;
import frc.robot.commands.Autonomous.autos.FiveBallAuto;
import frc.robot.commands.Autonomous.autos.FourBallAuto;
import frc.robot.commands.Autonomous.autos.OneBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAutoSimple;
import frc.robot.commands.triggers.AutoShoot;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystemNew;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    // Controllers and ports
    final Joystick driverJoystickForward = new Joystick(0);
    final Joystick driverJoystickTurn = new Joystick(1);
    final XboxController operatorController = new XboxController(2);

    private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    private DoubleSupplier throttle = () -> driverJoystickForward.getY();
    private DoubleSupplier turn = () -> driverJoystickTurn.getX() * 0.5;

    private final CompressorSubsystem compressor = new CompressorSubsystem();

    public final IntakeSubsystem frontIntake = new IntakeSubsystem(compressor, Intake.FRONT_INTAKE_PORT,
            Intake.FRONT_LEFT_SOLENOID_PORT, Intake.FRONT_RIGHT_SOLENOID_PORT);
    public final IntakeSubsystem rearIntake = new IntakeSubsystem(compressor, Intake.REAR_INTAKE_PORT,
            Intake.REAR_LEFT_SOLENOID_PORT, Intake.REAR_RIGHT_SOLENOID_PORT);

    private final boolean down = false;

    private final ChimneySubsystem chimney = new ChimneySubsystem();

    private final BooleanSupplier chimneyPower = () -> (driverJoystickForward.getTrigger()
            || driverJoystickTurn.getTrigger() || driverJoystickTurn.getTop());

    private final IndexerSubsystem indexer = new IndexerSubsystem();

    private final JoystickButton indexerUp = new JoystickButton(operatorController, Button.kX.value);
    private final JoystickButton indexerDown = new JoystickButton(operatorController, Button.kY.value);

    private final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();

    LimelightSubsystem limelight = new LimelightSubsystem();

    public final SeekCommand seekCommand = new SeekCommand(launcher, limelight, turret, false);

    JoystickButton doubleShoot = new JoystickButton(operatorController, Button.kRightBumper.value);
    JoystickButton doubleShotOverride = new JoystickButton(operatorController, Button.kLeftBumper.value);

    private final ClimberSubsystemNew climber = new ClimberSubsystemNew();

    final JoystickButton tippingToggle = new JoystickButton(operatorController, Button.kBack.value);

    public final Trigger autoShoot = new AutoShoot(() -> limelight.getLockedOn(), () -> launcher.isReady(),
            drivetrain.getAverageVelocity());

    // Auto Chooser
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putNumber("Launcher Velocity", 0.0);
        SmartDashboard.putNumber("Auto Duration", 0.0);

        // default commands for functions

        SmartDashboard.putBoolean("Aim Active", false);

        SmartDashboard.putString("Game Message", DriverStation.getGameSpecificMessage());

        setDefaultCommands();

        SetAutoCommands();

        // Restart Photon Vision
        try {
            Unirest.setTimeouts(0, 0);
            HttpResponse<String> response = Unirest.post("http://photonvision.local:5800/api/restartProgram")
                    .header("Content-Type", "text/plain")
                    .body("{}")
                    .asString();

            if (response.getStatus() != 200) {
                DriverStation.reportError("PhotonVision returned non 200 code", false);
            }

        } catch (Exception e) {
            DriverStation.reportError("Cannot restart Photon Vision", e.getStackTrace());
        }
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, throttle, turn));
        frontIntake.setDefaultCommand(
                new IntakeCommand(frontIntake, () -> driverJoystickForward.getTrigger(), down, true));
        rearIntake.setDefaultCommand(
                new IntakeCommand(rearIntake, () -> driverJoystickTurn.getTrigger(), down, true));
        chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower));
        indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0.0));
        launcher.setDefaultCommand(
                new LauncherCommand(launcher, () -> SmartDashboard.getNumber("LauncherVelocity", 0.0), () -> false));
        limelight.setDefaultCommand(new LimelightCommand(limelight));
        climber.setDefaultCommand(new TraversalClimbManualCommand(climber, () -> operatorController.getLeftY(),
                () -> operatorController.getRightY(), () -> operatorController.getYButton()));
        turret.setDefaultCommand(new TurretCommand(turret, () -> 0, () -> false, false, Turret.OFFSET_TICKS));
    }

    private void configureButtonBindings() {
        // backup kicker control if limelight fails
        indexerUp.whenHeld(new IndexerCommand(indexer, () -> 1));

        indexerDown.whenHeld(new IndexerCommand(indexer, () -> -1));

        // doubleShoot.and(autoShoot.negate())
        // .whenActive(seekCommand);

        doubleShoot.toggleWhenPressed(seekCommand);

        // doubleShoot.negate().cancelWhenActive(seekCommand);

        doubleShoot.and(autoShoot).or(doubleShotOverride).whenActive(
                new DoubleShotCommand(chimney, turret, indexer, launcher, limelight).withTimeout(1.3));

        tippingToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> drivetrain.toggleTippingEnabled(),
                        () -> drivetrain.toggleTippingDisabled()));

        SmartDashboard.putData("Seek Command", seekCommand);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command getTestAuto() {
        return null;
    }

    private void SetAutoCommands() {
        Command oneBallAuto = new OneBallAuto(drivetrain, indexer, launcher, limelight, turret);
        Command twoBallAuto = new TwoBallAuto(drivetrain, rearIntake, chimney, indexer, launcher, limelight, turret);
        Command fourBallAuto = new FourBallAuto(drivetrain, rearIntake, chimney, indexer, launcher, limelight, turret);
        Command fiveBallAuto = new FiveBallAuto(drivetrain, rearIntake, chimney, indexer, launcher, limelight, turret);

        Command nullAuto = null;

        Command testAuto = getTestAuto();

        Command simpleTwoBallAuto = new TwoBallAutoSimple(drivetrain, rearIntake, chimney, indexer, launcher, limelight,
                turret);

        autoChooser.addOption("One Ball", oneBallAuto);
        autoChooser.addOption("Two Ball", twoBallAuto);
        autoChooser.addOption("Two Ball Simple", simpleTwoBallAuto);
        autoChooser.addOption("Four Ball", fourBallAuto);
        autoChooser.setDefaultOption("Five Ball", fiveBallAuto);

        autoChooser.addOption("Test Auto", testAuto);
        autoChooser.addOption("No Auto", nullAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
