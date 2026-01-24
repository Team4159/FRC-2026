package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class ChangeLED extends Command {
    
    public LEDPattern pattern;

    private final LEDs led;
    private static Distance ledSpacing;
    
    public ChangeLED(LEDPattern pattern, LEDs led) {
        this.pattern = pattern;
        this.led = led;

        ledSpacing = Meters.of(1.0 / 120.0);


        addRequirements(led);
    }
    

    public static enum PatternList { 
        RAINBOW(LEDPattern.rainbow(255,64)),
        RAINBOW_SCROLL(RAINBOW.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing)),

        RED_SOLID(LEDPattern.solid(Color.kRed)),
        BLUE_SOLID(LEDPattern.solid(Color.kBlue)),
        GREEN_SOLID(LEDPattern.solid(Color.kGreen));

        private LEDPattern pattern;

        PatternList(LEDPattern c_InPattern) {
        }
    }

    public void execute() {
        pattern = PatternList.RAINBOW_SCROLL.pattern; //change once more options to not make input redundant

        led.setBuffer(pattern);
    }
    
}
