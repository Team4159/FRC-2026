package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    
    private static AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private static final int kPort = 8;
    private static final int kLength = 60;

    public LEDs () {
        led = new AddressableLED(kPort); //only ever one port so it could be static, but this is nicer i think.
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
