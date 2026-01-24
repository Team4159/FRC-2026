package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class ChangeLED extends Command {
    
    private LEDPattern pattern;

    private final LEDs led;
    private static Distance ledSpacing;
    private static LEDPattern rainbow = LEDPattern.rainbow(255, 64);
    
    enum PatternList { 
       rainbowScroll{
           public LEDPattern getPattern() {
               return rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
            }
       };

       public abstract LEDPattern getPattern(); 
    }
    
    public ChangeLED(LEDPattern pattern, LEDs led) {
        this.pattern = pattern;
        this.led = led;

        ledSpacing = Meters.of(1.0 / 120.0);


        addRequirements(led);
    }
    

    public void execute() {
        pattern = PatternList.rainbowScroll.getPattern(); //change once more options to not make input redundant

        led.setBuffer(pattern);
    }
    
}
