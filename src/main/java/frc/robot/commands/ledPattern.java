package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class ledPattern extends Command {
    private LEDPattern pattern;
    private LEDs led;
    private Distance ledSpacing;
    private LEDPattern rainbow = LEDPattern.rainbow(255, 64);
    private LEDPattern rainbowScroll =  rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
    
    public ledPattern(LEDPattern pattern, LEDs led) {
        this.pattern = pattern;
        this.led = led;

        ledSpacing = Meters.of(1 / 120);

        addRequirements(led);
    }
 
    public void execute() {
        pattern = rainbowScroll;

        led.setBuffer(pattern);
    }
    
}
