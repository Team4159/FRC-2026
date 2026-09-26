package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;
import java.util.Map;
import frc.lib.JoeLookupTable.LookupTablePoint;

public class JoeLookupTableConstants {

    /**
     * adjust the angle of the hood down by this much (in radians for each
     * meter/second slow the calculated tangential speed is
     * multiplied by the distance from the hub (higher distance needs more
     * correction)
     */
    public static final double SHOOTER_DISTANCE_VELOCITY_CORRECTION = 0.01;

    public static final Distance MAX_DISTANCE = Meters.of(4.5);

    // stores desired motor angular velocity and shooter efficiency based on position
    public static final Map<Distance, LookupTablePoint> JOE_LOOKUP_TABLE = Map.ofEntries(
        //non-continuous feeding
        // Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(2000), 0.93)),
        // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(2100), 0.92)),
        // Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2200), 0.91)),
        // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2300), 0.90)),
        // Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2400), 0.88)),
        // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2500), 0.84)),
        // Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(2700), 0.78)),
        // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2900), 0.72))
        //before gear ratio fix
        // Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(1800), 1.1)),
        // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 1.05)),
        // Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2000), 1.025)),
        // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 1)),
        // Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2200), 1)),
        // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.97)),
        // Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(2500), 0.90)),
        // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.85))
        // Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(1800), 0.92)),
        // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 0.90)),
        // Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2000), 0.88)),
        // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 0.85)),
        // Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2200), 0.80)),
        // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.78)),
        // Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(2500), 0.73)),
        // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.70))
        // Map.entry(Meters.of(1), new LookupTablePoint(RPM.of(1800), 1.20)), //1
        // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 1.20)),
        // Map.entry(Meters.of(2), new LookupTablePoint(RPM.of(2000), 1.15)), //2
        // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 1.05)),
        // Map.entry(Meters.of(3), new LookupTablePoint(RPM.of(2200), 1.00)), //3
        // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.95)),
        // Map.entry(Meters.of(4), new LookupTablePoint(RPM.of(2500), 0.90)),
        // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.85))
        Map.entry(Meters.of(1.0), new LookupTablePoint(RPM.of(1800), 1.1)),
        Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 1.05)),
        Map.entry(Meters.of(2.0), new LookupTablePoint(RPM.of(2000), 1.025)),
        Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 1)),
        Map.entry(Meters.of(3.0), new LookupTablePoint(RPM.of(2200), 1)),
        Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.97)),
        Map.entry(Meters.of(4.0), new LookupTablePoint(RPM.of(2500), 0.90)),
        Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.85))
    );
}
