// JA 4/13 11:30am changed:
// ClimberSubsystem to ClimberSubsystemNew
// ClimberCommand to ClimberCommandNew
// Added ClimberButtonAnalog

package frc.robot;

import java.util.function.DoubleSupplier;

import com.mashape.unirest.http.HttpResponse;
import com.mashape.unirest.http.Unirest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Intake;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.TraversalClimbManualCommand;
import frc.robot.commands.TurretPowerCommand;
import frc.robot.commands.AutomatedCommands.DoubleShotCommand;
import frc.robot.commands.AutomatedCommands.RejectBallCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    // Controllers and ports
    private final XboxController driverController = new XboxController(0);

    // initializing subsystems
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final ChimneySubsystem chimney = new ChimneySubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final LauncherSubsystem launcher = new LauncherSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    public final IntakeSubsystem frontIntake = new IntakeSubsystem(Intake.FRONT_INTAKE_PORT,
            Intake.FRONT_LEFT_SOLENOID_PORT, Intake.FRONT_RIGHT_SOLENOID_PORT);

    // drive controls
    private DoubleSupplier throttle = () -> driverController.getLeftY();
    private DoubleSupplier turn = () -> driverController.getRightX() * 0.5;

    // chimney controls
    private final DoubleSupplier chimneyPower = () -> (driverController.getAButton()) ? 1 : 0;

    // private final DoubleSupplier climberPowerAnalog = () ->
    // -operatorController.getLeftTriggerAxis()
    // + -operatorController.getRightTriggerAxis();

    // manual kicker controls
    private final JoystickButton kickerUp = new JoystickButton(driverController, Button.kX.value);
    private final JoystickButton kickerDown = new JoystickButton(driverController, Button.kY.value);

    // seek button
    private final JoystickButton turretRight = new JoystickButton(driverController, Button.kRightBumper.value);

    private final JoystickButton turretLeft = new JoystickButton(driverController, Button.kLeftBumper.value);

    // don't shoot button
    private final JoystickButton shootButton = new JoystickButton(driverController, Button.kB.value);

    // reject ball button
    private final JoystickButton rejectBall = new JoystickButton(driverController, Button.kLeftStick.value);

    // toggles drivetrain roll protection
    private final JoystickButton tippingToggle = new JoystickButton(driverController, Button.kBack.value);

    private final JoystickButton engageFlywheels = new JoystickButton(driverController, Button.kStart.value);

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

        // SetAutoCommands();

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
                new IntakeCommand(frontIntake, () -> driverController.getAButton() ? -0.8 : 0, false));
        chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower));
        kicker.setDefaultCommand(new KickerCommand(kicker, () -> 0.0));
        launcher.setDefaultCommand(
                new LauncherCommand(launcher, () -> SmartDashboard.getNumber("LauncherVelocity", 0.0), () -> false));
        climber.setDefaultCommand(
                new TraversalClimbManualCommand(climber,
                        () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
                        () -> driverController.getRightTriggerAxis(), () -> driverController.getPOV() == 0));
        turret.setDefaultCommand(new TurretPowerCommand(turret, () -> 0));
    }

    private void configureButtonBindings() {
        // backup kicker control if limelight fails
        kickerUp.whenHeld(new KickerCommand(kicker, () -> 1));

        kickerDown.whenHeld(new KickerCommand(kicker, () -> -1));

        turretLeft.whenHeld(new TurretPowerCommand(turret, () -> -0.6));
        turretRight.whenHeld(new TurretPowerCommand(turret, () -> 0.6));

        shootButton.whenActive(
                new DoubleShotCommand(chimney, turret, kicker, launcher).withTimeout(1.3));

        rejectBall.whenActive(new RejectBallCommand(chimney, turret, kicker, launcher));

        tippingToggle.toggleWhenPressed(
                new StartEndCommand(
                        () -> drivetrain.toggleTippingDisabled(),
                        () -> drivetrain.toggleTippingEnabled()));

        engageFlywheels.toggleWhenPressed(new LauncherCommand(launcher, () -> 10000, () -> true, true));
    }
}
