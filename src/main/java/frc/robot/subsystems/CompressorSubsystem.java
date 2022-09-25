package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
    // creates an instance of a compressor
    private Compressor compressor;

    // this is the counter for the compressor subsystem
    // counts up when the compressor is used
    // counts down when the compressor stops being used
    private int counter = 0;

    /**
     * Compressor subsystem constructor
     */
    public CompressorSubsystem() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    /**
     * Increments the counter by 1
     */
    public void countUp() {
        counter++;
        if (counter > 0) {
            enableCompressor();
        }
    }

    /**
     * Decrements the counter by 1
     */
    public void countDown() {
        counter--;
        if (counter <= 0) {
            counter = 0;
            disableCompressor();
        }
    }

    /**
     * Disables the compressor
     */
    public void disableCompressor() {
        compressor.disable();
    }

    /**
     * Enables the compressor
     */
    public void enableCompressor() {
        compressor.enableDigital();
    }

    /**
     * 
     * @return the value of the counter
     */
    public int getCount() {
        return counter;
    }
}
