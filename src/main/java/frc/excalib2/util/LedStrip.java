package frc.excalib2.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Addressable LED strip built on WPILib's {@link LEDPattern} library (solid, blink,
 * rainbow, gradients, progress bars — compose them with the pattern API).
 *
 * <p>Port of ExcaLib v1's {@code LEDs}, minus the singleton and the hand-rolled pattern
 * engine: construct one where the robot owns it and bind patterns with
 * {@link #runPatternCommand}. A default pattern shows whenever no command holds the strip.
 *
 * <pre>{@code
 * LedStrip leds = new LedStrip(8, 22);
 * leds.setDefaultPattern(LEDPattern.solid(Color.kBlue));
 * shooterSpinning.whileTrue(leds.runPatternCommand(
 *         LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.2))));
 * }</pre>
 */
public class LedStrip extends SubsystemBase {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;
    private LEDPattern defaultPattern = LEDPattern.solid(Color.kBlack);
    private LEDPattern currentPattern = defaultPattern;

    public LedStrip(int pwmPort, int length) {
        strip = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(length);
        strip.setLength(length);
        strip.start();
    }

    /** Pattern shown whenever no command is running on the strip. */
    public void setDefaultPattern(LEDPattern pattern) {
        boolean wasDefault = currentPattern == defaultPattern;
        defaultPattern = pattern;
        if (wasDefault) {
            currentPattern = pattern;
        }
    }

    /** Shows {@code pattern} while scheduled, restoring the default on end. Runs while disabled. */
    public Command runPatternCommand(LEDPattern pattern) {
        return startEnd(
                () -> currentPattern = pattern,
                () -> currentPattern = defaultPattern)
                .ignoringDisable(true)
                .withName("LED Pattern");
    }

    @Override
    public void periodic() {
        currentPattern.applyTo(buffer);
        strip.setData(buffer);
    }
}
