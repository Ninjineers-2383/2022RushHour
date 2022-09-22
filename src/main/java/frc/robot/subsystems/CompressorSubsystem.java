package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private int counter = 0;

    public CompressorSubsystem() {
    }

    public void countUp() {
        counter++;
    }

    public void countDown() {
        counter--;
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public int getCount() {
        return counter;
    }
}
