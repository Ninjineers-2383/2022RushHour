package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.DoubleIntakeCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LauncherCommand;

public class ColorSensorSubsystem extends SubsystemBase {

    // create motor instance that uses a TalonSRX motor controller.
    private final ColorSensorV3 colorSensor;
    private final IntakeSubsystem intake;
    private final ChimneySubsystem chimney;
    private final IndexerSubsystem indexer;
    private final LauncherSubsystem launcher;
    private boolean active; // pretty much if auto is finished or not. no pooping during auto

    // // Chimney subsystem constructor
    public ColorSensorSubsystem(IntakeSubsystem intake, ChimneySubsystem chimney) {
        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        this.intake = intake;
        this.chimney = chimney;
        this.launcher = null;
        this.indexer = null;
        active = false;
    }

    public ColorSensorSubsystem(LauncherSubsystem launcher, IndexerSubsystem indexer) {
        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        this.intake = null;
        this.chimney = null;
        this.indexer = indexer;
        this.launcher = launcher;
        active = false;
    }

    public void setActiveTrue() {
        active = true;
    }

    public void setActiveFalse() {
        active = false;
    }

    public boolean getActive() {
        return active;
    }

    public Alliance getTeamColor() {
        // return teamColor;
        return DriverStation.getAlliance();
    }

    public Alliance getOppositeTeamColor() {
        // return teamColor.equals("red") ? "blue" : "red";
        return DriverStation.getAlliance() == Alliance.Red ? Alliance.Blue : Alliance.Red;
    }

    // int 0
    public Alliance colorCheck() {
        int distance = colorSensor.getProximity();
        // int red = colorSensor.getRed();
        int blue = colorSensor.getBlue();

        Alliance detectedColor = Alliance.Invalid;

        if (distance > 70 && active) {
            if (blue < 185) {
                detectedColor = Alliance.Red;
                // System.out.println("red");
            } else {
                detectedColor = Alliance.Blue;
                // System.out.println("blue");
            }
        }

        SmartDashboard.putString("detectedColor", detectedColor.toString());
        return detectedColor;
    }

    public Command loadIn(boolean frontDown, boolean rearDown) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new DoubleIntakeCommand(intake, () -> -1, () -> -1).withTimeout(0.5),
                        new ChimneyCommand(chimney, () -> -1)),
                new ParallelRaceGroup(
                        new DoubleIntakeCommand(intake, () -> 0, () -> 0).withTimeout(0.05),
                        new ChimneyCommand(chimney, () -> 0)));
    }

    // true means front down, false means back.
    public Command loadOut(BooleanSupplier frontDown) {
        return new ParallelRaceGroup(
                new DoubleIntakeCommand(intake, () -> (frontDown.getAsBoolean() ? 1 : -1),
                        () -> (frontDown.getAsBoolean() ? -1 : 1)).withTimeout(2),
                new ChimneyCommand(chimney, () -> 0.4));
    }

    public Command shootOut() {
        return new ParallelRaceGroup(
                new LauncherCommand(launcher, () -> 4000),
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        // new ParallelRaceGroup(
                        new IndexerCommand(indexer, () -> 1).withTimeout(1)
                // new ChimneyCommand(chimney, () -> -0.5)
                ));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ir", colorSensor.getIR());
        SmartDashboard.putNumber("proximity", colorSensor.getProximity());
        SmartDashboard.putNumber("red", colorSensor.getRed());
        SmartDashboard.putNumber("blue", colorSensor.getBlue());
        SmartDashboard.putNumber("green", colorSensor.getGreen());
        SmartDashboard.putBoolean("is connected", colorSensor.isConnected());
        colorCheck();
        SmartDashboard.putBoolean("Pooper Active", active);
    }

}
