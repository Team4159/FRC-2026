package frc.lib;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.Constants.PhysicsConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/*
simplified version of FuelSim by Team 5000
https://github.com/hammerheads5000/FuelSim
*/
public class FuelSimulation {

    private static final double SIMULATION_STEP_PERIOD = 0.005;
    private static final int SIMULATION_MAX_STEPS_PER_FRAME = 20;
    private static final double SIMULATION_TIME_FACTOR = 1.0;
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -PhysicsConstants.GRAVITY);
    private static final double AIR_DENSITY = 1.2;
    private static final double FUEL_RADIUS = 0.15;
    // private static final double FUEL_MASS = Units.lbsToKilograms((0.5 + 0.448) / 2.0);
    private static final double FUEL_SPACING = Units.inchesToMeters(6.0);
    private static final double FUEL_CROSS_SECTIONAL_AREA = Math.PI * Math.pow(FUEL_RADIUS, 2);
    private static final double FUEL_DRAG_COEFFICIENT = 0.47; // of a sphere
    private static final Translation3d FIELD_CENTER = new Translation3d(
        Units.inchesToMeters(651.22 / 2.0),
        Units.inchesToMeters(317.69 / 2.0),
        0.0
    );
    private static final double FIELD_CENTER_FUEL_OFFSET = Units.inchesToMeters(0.95) + FUEL_RADIUS;

    private static final double SHOT_FUEL_LIFETIME = 5.0;

    private final ArrayList<Fuel> fuels = new ArrayList<>();
    private final Map<Fuel, Double> shotFuelTimestamps = new HashMap<>();

    private final StructArrayPublisher<Translation3d> fuelSimulationPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Fuel Simulation", Translation3d.struct)
        .publish();

    private static FuelSimulation instance;

    private class Fuel {

        @SuppressWarnings("unused")
        private Translation3d position, linearVelocity, angularVelocity;

        private double accumulatedDeltaTime = 0.0;

        private Fuel(Translation3d position, Translation3d linearVelocity, Translation3d angularVelocity) {
            this.position = position;
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
            fuels.add(this);
        }

        private Fuel(Translation3d position) {
            this(position, new Translation3d(0, 0, 0), new Translation3d(0, 0, 0));
        }

        private void update(double deltaTime) {
            accumulatedDeltaTime += deltaTime;
            int steps = (int) (accumulatedDeltaTime / SIMULATION_STEP_PERIOD);
            accumulatedDeltaTime %= SIMULATION_STEP_PERIOD;
            for (int i = 0; i < steps; i++) {
                stepPhysics(SIMULATION_STEP_PERIOD * SIMULATION_TIME_FACTOR);
            }
        }

        private void stepPhysics(double deltaTime) {
            if (position.getZ() > FUEL_RADIUS || linearVelocity.getZ() > 0) {
                double linearVelocityMagnitude = linearVelocity.getNorm();
                Vector<N3> linearVector = linearVelocity.toVector();
                Vector<N3> linearUnitVector = linearVector.unit();
                // gravity
                linearVelocity = linearVelocity.plus(GRAVITY.times(deltaTime));
                // air resistance
                double airResistanceMagnitude =
                    0.5 *
                    FUEL_DRAG_COEFFICIENT *
                    AIR_DENSITY *
                    FUEL_CROSS_SECTIONAL_AREA *
                    Math.pow(linearVelocityMagnitude, 2);
                @SuppressWarnings("unused")
                Translation3d airResistanceForce = new Translation3d(linearUnitVector.times(airResistanceMagnitude));
                //linearVelocity = linearVelocity.minus(airResistanceForce.div(kFuelMass).times(deltaTime));
            } else {
                position = new Translation3d(position.getX(), position.getY(), FUEL_RADIUS);
                linearVelocity = new Translation3d(0, 0, 0);
                angularVelocity = new Translation3d(0, 0, 0);
            }
            position = position.plus(linearVelocity.times(deltaTime));
        }

        private void destroy() {
            fuels.remove(this);
        }
    }

    private double lastUpdate;

    private FuelSimulation() {
        lastUpdate = getTime();
    }

    public static FuelSimulation getInstance() {
        if (instance == null) {
            instance = new FuelSimulation();
        }
        return instance;
    }

    public void setupFieldFuel() {
        for (int x = -6; x < 6; x++) {
            for (int y = -14; y <= 0; y++) {
                new Fuel(
                    FIELD_CENTER.plus(
                        new Translation3d(
                            x * FUEL_SPACING + FUEL_RADIUS,
                            y * FUEL_SPACING - FIELD_CENTER_FUEL_OFFSET,
                            FUEL_RADIUS
                        )
                    )
                );
            }
            for (int y = 0; y <= 14; y++) {
                new Fuel(
                    FIELD_CENTER.plus(
                        new Translation3d(
                            x * FUEL_SPACING + FUEL_RADIUS,
                            y * FUEL_SPACING + FIELD_CENTER_FUEL_OFFSET,
                            FUEL_RADIUS
                        )
                    )
                );
            }
        }
    }

    public void shootFuel(Translation3d position, Translation3d linearVelocity, Translation3d angularVelocity) {
        Translation3d correctedPosition = new Translation3d(
            position.getX(),
            position.getY(),
            Math.max(position.getZ(), FUEL_RADIUS)
        );
        Fuel fuel = new Fuel(correctedPosition, linearVelocity, angularVelocity);
        shotFuelTimestamps.put(fuel, getTime());
    }

    public void output() {
        fuelSimulationPublisher.set(
            fuels
                .stream()
                .map(fuel -> fuel.position)
                .toArray(Translation3d[]::new)
        );
    }

    public void update() {
        double time = getTime();
        // delete shot fuels
        var iterator = shotFuelTimestamps.entrySet().iterator();
        while (iterator.hasNext()) {
            var entry = iterator.next();
            var fuel = entry.getKey();
            var spawnTime = entry.getValue();
            if (time - spawnTime < SHOT_FUEL_LIFETIME) {
                continue;
            }
            fuel.destroy();
            iterator.remove();
        }
        // step physics
        double deltaTime = Math.min(time - lastUpdate, SIMULATION_STEP_PERIOD * SIMULATION_MAX_STEPS_PER_FRAME);
        for (Fuel fuel : fuels) {
            fuel.update(deltaTime);
        }
        lastUpdate = time;
        output();
    }

    private double getTime() {
        return MathSharedStore.getTimestamp();
    }
}
