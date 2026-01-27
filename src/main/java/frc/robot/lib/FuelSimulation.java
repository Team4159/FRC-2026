package frc.robot.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

/*
simplified version of FuelSim by Team 5000 that includes nothing
https://github.com/hammerheads5000/FuelSim
*/
public class FuelSimulation {
    private static final double kSimulationStepPeriod = 0.005;
    private static final int kSimulationMaxStepsPerFrame = 20;
    private static final double kSimulationTimeScale = 1.0;
    private static final Translation3d kGravity = new Translation3d(0, 0, -FieldConstants.g);
    private static final double kAirDensity = 1.2;
    private static final double kFuelRadius = 0.15;
    private static final double kFuelMass = Units.lbsToKilograms((0.5 + 0.448) / 2.0);
    private static final double kFuelSpacing = Units.inchesToMeters(6.0);
    private static final double kFuelCrossSectionalArea = Math.PI * Math.pow(kFuelRadius, 2);
    private static final double kFuelDragCoefficient = 0.47; // of a sphere
    private static final Translation3d kFieldCenter = new Translation3d(
        Units.inchesToMeters(651.22 / 2.0),
        Units.inchesToMeters(317.69 / 2.0),
        0
    );
    private static final double kFieldCenterFuelOffset = Units.inchesToMeters(0.95) + kFuelRadius;

    private static final double kShotFuelLifetime = 5.0;

    private static final ArrayList<Fuel> fuels = new ArrayList<>();
    private static final Map<Fuel, Double> shotFuelTimestamps = new HashMap<>();

    private static FuelSimulation instance;

    private static class Fuel {
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
            int steps = (int)(accumulatedDeltaTime / kSimulationStepPeriod);
            accumulatedDeltaTime %= kSimulationStepPeriod;
            for (int i = 0; i < steps; i++) {
                stepPhysics(kSimulationStepPeriod * kSimulationTimeScale);
            }
        }

        private void stepPhysics(double deltaTime) {
            if (position.getZ() > kFuelRadius || linearVelocity.getZ() > 0) {
                double linearVelocityMagnitude = linearVelocity.getNorm();
                Vector<N3> linearVector = linearVelocity.toVector();
                Vector<N3> linearUnitVector = linearVector.unit();
                // gravity
                linearVelocity = linearVelocity.plus(kGravity.times(deltaTime));
                // air resistance
                double airResistanceMagnitude = 0.5 * kFuelDragCoefficient * kAirDensity * kFuelCrossSectionalArea * Math.pow(linearVelocityMagnitude, 2);
                Translation3d airResistanceForce = new Translation3d(linearUnitVector.times(airResistanceMagnitude));
                //linearVelocity = linearVelocity.minus(airResistanceForce.div(kFuelMass).times(deltaTime));
            } else {
                position = new Translation3d(position.getX(), position.getY(), kFuelRadius);
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
                new Fuel(kFieldCenter.plus(new Translation3d(x * kFuelSpacing + kFuelRadius, y * kFuelSpacing - kFieldCenterFuelOffset, kFuelRadius)));
            }
            for (int y = 0; y <= 14; y++) {
                new Fuel(kFieldCenter.plus(new Translation3d(x * kFuelSpacing + kFuelRadius, y * kFuelSpacing + kFieldCenterFuelOffset, kFuelRadius)));
            }
        }
    }

    public void shootFuel(Translation3d position, Translation3d linearVelocity, Translation3d angularVelocity) {
        Translation3d correctedPosition = new Translation3d(position.getX(), position.getY(), Math.max(position.getZ(), kFuelRadius));
        Fuel fuel = new Fuel(correctedPosition, linearVelocity, angularVelocity);
        shotFuelTimestamps.put(fuel, getTime());
    }

    public void output() {
        Logger.recordOutput("Fuel Simulation/Fuels", fuels.stream().map((fuel) -> fuel.position).toArray(Translation3d[]::new));
    }

    public void update() {
        double time = getTime();
        // delete shot fuels
        var iterator = shotFuelTimestamps.entrySet().iterator();
        while (iterator.hasNext()) {
            var entry = iterator.next();
            var fuel = entry.getKey();
            var spawnTime = entry.getValue();
            if (time - spawnTime < kShotFuelLifetime) { continue; }
            fuel.destroy();
            iterator.remove();
        }
        // step physics
        double deltaTime = Math.min(time - lastUpdate, kSimulationStepPeriod * kSimulationMaxStepsPerFrame);
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