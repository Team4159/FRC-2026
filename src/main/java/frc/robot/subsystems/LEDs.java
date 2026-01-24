package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private static final int kPort = 8;
    private static final int kLength = 60;

    private static final AddressableLED led = new AddressableLED(kPort); //i would prefer to not make it static and define it in the constructor but ethan thinks this is better.
    private final AddressableLEDBuffer ledBuffer;


    public LEDs () {
        ledBuffer = new AddressableLEDBuffer(kLength);

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();
    }

    public void setBuffer(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
