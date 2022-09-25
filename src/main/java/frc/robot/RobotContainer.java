// JA 4/13 11:30am changed:
// ClimberSubsystem to ClimberSubsystemNew
// ClimberCommand to ClimberCommandNew
// Added ClimberButtonAnalog

package frc.robot;

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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TraversalClimbManualCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.AutomatedCommands.DoubleShotCommand;
import frc.robot.commands.AutomatedCommands.SeekCommand;
import frc.robot.commands.AutomatedCommands.StopLaunchCommand;
import frc.robot.commands.Autonomous.autos.FiveBallAuto;
import frc.robot.commands.Autonomous.autos.FourBallAuto;
import frc.robot.commands.Autonomous.autos.OneBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAuto;
import frc.robot.commands.Autonomous.autos.TwoBallAutoSimple;
import frc.robot.commands.triggers.AutoShoot;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    // Controllers and ports
    private final Joystick driverJoystickForward = new Joystick(0);
    private final Joystick driverJoystickTurn = new Joystick(1);
    private final XboxController operatorController = new XboxController(2);

    // initializing subsystems
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final CompressorSubsystem compressor = new CompressorSubsystem();
    private final ChimneySubsystem chimney = new ChimneySubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    private final LimelightSubsystem limelight = new LimelightSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    public final IntakeSubsystem frontIntake = new IntakeSubsystem(compressor, Intake.FRONT_INTAKE_PORT,
            Intake.FRONT_LEFT_SOLENOID_PORT, Intake.FRONT_RIGHT_SOLENOID_PORT);

    public final IntakeSubsystem rearIntake = new IntakeSubsystem(compressor, Intake.REAR_INTAKE_PORT,
            Intake.REAR_LEFT_SOLENOID_PORT, Intake.REAR_RIGHT_SOLENOID_PORT);

    // drive controls
    private DoubleSupplier throttle = () -> driverJoystickForward.getY();
    private DoubleSupplier turn = () -> driverJoystickTurn.getX() * 0.5;

    // chimney controls
    private final DoubleSupplier chimneyPower = () -> (driverJoystickForward.getTrigger()
            || driverJoystickTurn.getTrigger() || driverJoystickTurn.getTop()) ? 1 : 0;

    // manual kicker controls
    private final JoystickButton kickerUp = new JoystickButton(operatorController, Button.kX.value);
    private final JoystickButton kickerDown = new JoystickButton(operatorController, Button.kY.value);

    // seek button
    private final JoystickButton seekButton = new JoystickButton(operatorController, Button.kRightBumper.value);

    // don't shoot button
    private final JoystickButton doNotShootButton = new JoystickButton(operatorController, Button.kLeftBumper.value);

    // sets drivetrain and climber to coast
    private final JoystickButton coastToggle = new JoystickButton(operatorController, Button.kBack.value);

    // cancels the seek command
    private final JoystickButton cancelSeek = new JoystickButton(operatorController, Button.kStart.value);

    // determines whether or not to shoot
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
                new IntakeCommand(frontIntake, () -> driverJoystickForward.getTrigger() ? -0.8 : 0, false));
        rearIntake.setDefaultCommand(
                new IntakeCommand(rearIntake, () -> driverJoystickTurn.getTrigger() ? -0.8 : 0, false));
        chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower));
        kicker.setDefaultCommand(new KickerCommand(kicker, () -> 0.0));
        launcher.setDefaultCommand(
                new LauncherCommand(launcher, () -> SmartDashboard.getNumber("LauncherVelocity", 0.0), () -> false));
        limelight.setDefaultCommand(new LimelightCommand(limelight));
        climber.setDefaultCommand(new TraversalClimbManualCommand(climber, () -> operatorController.getLeftY(),
                () -> operatorController.getRightY(), () -> operatorController.getYButton()));
        turret.setDefaultCommand(new TurretCommand(turret, () -> 0, () -> false, false, Turret.OFFSET_TICKS));
    }

    private void configureButtonBindings() {
        // backup kicker control if limelight fails
        kickerUp.whenHeld(new KickerCommand(kicker, () -> 1));

        kickerDown.whenHeld(new KickerCommand(kicker, () -> -1));

        seekButton.toggleWhenPressed(new SeekCommand(launcher, limelight, turret, false));

        cancelSeek.whileHeld(new StopLaunchCommand(launcher, kicker, chimney, turret));

        autoShoot.and(doNotShootButton.negate()).whenActive(
                new DoubleShotCommand(chimney, turret, kicker, launcher, limelight).withTimeout(1.3));

        coastToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> drivetrain.toggleTippingEnabled(),
                        () -> drivetrain.toggleTippingDisabled()));

        SmartDashboard.putData("Seek Command", new SeekCommand(launcher, limelight, turret, false));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command getTestAuto() {
        return null;
    }

    private void SetAutoCommands() {
        Command oneBallAuto = new OneBallAuto(drivetrain, kicker, launcher, limelight, turret);
        Command twoBallAuto = new TwoBallAuto(drivetrain, frontIntake, rearIntake, chimney, kicker, launcher,
                limelight, turret);
        Command fourBallAuto = new FourBallAuto(drivetrain, frontIntake, rearIntake, chimney, kicker, launcher,
                limelight, turret);
        Command fiveBallAuto = new FiveBallAuto(drivetrain, frontIntake, rearIntake, chimney, kicker, launcher,
                limelight, turret);

        Command nullAuto = null;

        Command testAuto = getTestAuto();

        Command simpleTwoBallAuto = new TwoBallAutoSimple(drivetrain, frontIntake, rearIntake, chimney, kicker,
                launcher, limelight,
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
