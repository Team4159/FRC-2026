package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    public LEDs () {
        led = new AddressableLED(8);
        ledBuffer = new AddressableLEDBuffer(60);

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();
    }

    public void setBuffer(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
