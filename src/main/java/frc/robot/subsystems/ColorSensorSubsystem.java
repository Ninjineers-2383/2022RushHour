package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.DoubleIntakeCommand;


public class ColorSensorSubsystem extends SubsystemBase {

    // create motor instance that uses a TalonSRX motor controller.
    private final ColorSensorV3 colorSensor;
    private String teamColor;
    private final IntakeSubsystem intake;
    private final ChimneySubsystem chimney;
    private boolean active; //pretty much if auto is finished or not. no pooping during auto
    
    
    // Chimney subsystem constructor
    public ColorSensorSubsystem(IntakeSubsystem intake, ChimneySubsystem chimney) {
        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        this.intake = intake;
        this.chimney = chimney;
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

    public void setColor(String color) {
        teamColor = color;
    }

    public String getTeamColor() {
        // return teamColor;
        return "red";
    }

    public String getOppositeTeamColor() {
        // return teamColor.equals("red") ? "blue" : "red";
        return "blue";
    }


    //int 0 
    public String colorCheck() {
        int distance = colorSensor.getProximity();
        int red = colorSensor.getRed();
        int blue = colorSensor.getBlue();
        // System.out.println(teamColor);
        String detectedColor = "";
        
        if (distance > 50 && active) {
            if(red > blue) {
                detectedColor = "red";
                //System.out.println("red");
            } else {
                detectedColor = "blue";
                //System.out.println("blue");
            }
        }
        else {
            detectedColor = "none";
        }

        SmartDashboard.putString("detectedColor", detectedColor);
        return detectedColor;
    }

    public Command loadIn(boolean frontDown, boolean rearDown) {
        double p = 1;
        return new SequentialCommandGroup(
            new DoubleIntakeCommand(intake, ()-> -1,() -> -1).withTimeout(0.1 * p),
            new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.5 * p),
            new DoubleIntakeCommand(intake, ()-> 0,() -> 0).withTimeout(0.05 * p),
            new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.05 * p)
        );
    }

    //true means front down, false means back.
    public Command loadOut(BooleanSupplier frontDown) {
        double p = 10;        
        return new ParallelCommandGroup(
                new DoubleIntakeCommand(intake, ()-> (frontDown.getAsBoolean() ? 1:-1) ,() -> (frontDown.getAsBoolean() ? -1:1)).withTimeout(0.1 * p),
                new ChimneyCommand(chimney, ()-> 0.2, intake).withTimeout(0.05 * p)
            );
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
