package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class ChangeLED extends Command {
    
    private LEDPattern pattern;

    private final LEDs led;
    private static Distance ledSpacing;
    
    public static enum LEDStatus { 
        RAINBOW(LEDPattern.rainbow(255,64)),
        RAINBOW_SCROLL(RAINBOW.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing)),

        RED_SOLID(LEDPattern.solid(Color.kRed)),
        BLUE_SOLID(LEDPattern.solid(Color.kBlue)),
        GREEN_SOLID(LEDPattern.solid(Color.kGreen));

        private LEDPattern pattern;

        LEDStatus(LEDPattern c_InPattern) {
            this.pattern = c_InPattern;
        }

        //@Override //gives error, not completely necessary too since method does not override anything.
        public LEDPattern getPattern() {
            return pattern;
        }
    }

    public ChangeLED(LEDStatus ledStatus, LEDs led) {
        this.pattern = ledStatus.getPattern();
        this.led = led;

        ledSpacing = Meters.of(1.0 / 120.0);


        addRequirements(led);
    }
    
    public void execute() {
        led.setBuffer(pattern);
    }
    
}
