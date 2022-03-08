package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.DoubleIntakeCommand;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.util.Color;


public class ColorSensorSubsystem extends SubsystemBase {

    // create motor instance that uses a TalonSRX motor controller.
    private final ColorSensorV3 colorSensor;
    private String teamColor;
    private final IntakeSubsystem intake;
    private final ChimneySubsystem chimney;
    
    
    // Chimney subsystem constructor
    public ColorSensorSubsystem(IntakeSubsystem intake, ChimneySubsystem chimney) {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        this.intake = intake;
        this.chimney = chimney;
    }

    public void setColor(String color) {
        teamColor = color;
    }

    public String getTeamColor() {
        return teamColor;
    }

    //int 0 
    public String colorCheck() {
        int red = colorSensor.getRed();
        int blue = colorSensor.getBlue();
        // System.out.println(teamColor);
        String detectedColor = "";
        
        if(red > 250) {
            detectedColor = "red";
            //System.out.println("red");
        } else if (blue > 200) {
            detectedColor = "blue";
            //System.out.println("blue");
        } else {
            detectedColor = "nothing";
        }

        SmartDashboard.putString("detectedColor", detectedColor);
        return detectedColor;
    }

    public Command loadIn() {
        double p = 1;
        return new SequentialCommandGroup(
            new DoubleIntakeCommand(intake, ()-> -1,() -> -1, true, false).withTimeout(0.1 * p),
            new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.5 * p),
            new DoubleIntakeCommand(intake, ()-> 0,() -> 0, true, false).withTimeout(0.05 * p),
            new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.05 * p)
        );
    }

    //true means front down, false means back.
    public Command loadOut(boolean front) {
        double p = 200;
        if(front) {
            return new SequentialCommandGroup(
                new DoubleIntakeCommand(intake, ()-> -1,() -> 1, true, false).withTimeout(0.1 * p),
                new DoubleIntakeCommand(intake, ()-> 0,() -> 0, true, false).withTimeout(0.05 * p)
            );
        } else {
            return new SequentialCommandGroup(
                new DoubleIntakeCommand(intake, ()-> 1,() -> -1, true, false).withTimeout(0.1 * p),
                new DoubleIntakeCommand(intake, ()-> 0,() -> 0, true, false).withTimeout(0.05 * p)
            );
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("proximity", colorSensor.getProximity());
        SmartDashboard.putNumber("red", colorSensor.getRed());
        SmartDashboard.putNumber("blue", colorSensor.getBlue());
        SmartDashboard.putNumber("green", colorSensor.getGreen());
        SmartDashboard.putBoolean("is connected", colorSensor.isConnected());
        // if (colorCheck() != null) {
        //     CommandScheduler.getInstance().schedule(colorCheck());
        // }
    }

}
