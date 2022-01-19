package frc.robot.ninjaLib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPadButtons extends Button {

    Joystick m_joystick;
    Direction m_direction;

    public DPadButtons(Joystick joystick, Direction direction) {
        this.m_joystick = joystick;
        this.m_direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int directionto;

        private Direction(int direction) {
            this.directionto = direction;
        }
    }

    public boolean get() {
        int dPadValue = m_joystick.getPOV();
        return (dPadValue == m_direction.directionto);
    }

}
