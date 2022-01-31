package frc.robot.commands;

import frc.robot.Constants.Turret;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TurretCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final TurretSubsystem m_subsystem;
    private final DoubleSupplier m_speed;
    private boolean side = false;


    public TurretCommand(TurretSubsystem subsystem, DoubleSupplier speed) {
        m_subsystem = subsystem;
        m_speed = speed;
        addRequirements(subsystem);
    } 


    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1 degree of rotation = 145.695364 ticks
        if (m_subsystem.getCurrentPosition() > Turret.TURRET_INBOUNDS) {
            SmartDashboard.putString("Hello", "am Turning");
            m_subsystem.turn(0.2);
            side = true;
        } else if (m_subsystem.getCurrentPosition() < -Turret.TURRET_INBOUNDS) {
            SmartDashboard.putString("Wassup", "Am also turning");
            m_subsystem.turn(-0.2);
            side = false;
        } else {
            SmartDashboard.putString("fUcK yOu", "Brian suck a dik");
            m_subsystem.turn(m_speed.getAsDouble());
        }
    }

    //Seeking side
    public boolean getSide(){
        return side;
    }
}
