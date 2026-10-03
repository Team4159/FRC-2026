package frc.lib;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class Telemetry {

    private static enum ElectricityCategory {
        SWERVE_DRIVE,
        SWERVE_STEER,
        FLYWHEEL,
        HOOD,
        NECK,
        HOPPER,
        INTAKE_PIVOT,
        INTAKE_ROLLER,
    }

    private final double JOULES_TO_WATT_HOURS = 1.0 / 3600.0;

    private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    private final NetworkTable electricityTable = networkTableInstance.getTable("Electricity");
    private final DoublePublisher batteryVoltagePublisher = electricityTable
        .getDoubleTopic("Battery Voltage (V)")
        .publish();
    private final DoublePublisher totalEnergyPublisher = electricityTable.getDoubleTopic("Total Energy (Wh)").publish();
    private final DoublePublisher totalCurrentPublisher = electricityTable
        .getDoubleTopic("Total Current (A)")
        .publish();
    private final DoubleArrayPublisher allChannelCurrentsPublisher = electricityTable
        .getDoubleArrayTopic("All Channel Currents (A)")
        .publish();
    private final NetworkTable energyBreakdownTable = electricityTable.getSubTable("Energy Breakdown");
    private final Map<ElectricityCategory, DoublePublisher> energyBreakdownPublishers = new HashMap<
        ElectricityCategory,
        DoublePublisher
    >();
    private final Map<ElectricityCategory, Double> energyBreakdown = new HashMap<ElectricityCategory, Double>();
    private final NetworkTable currentBreakdownTable = electricityTable.getSubTable("Current Breakdown");
    private final Map<ElectricityCategory, DoublePublisher> currentBreakdownPublishers = new HashMap<
        ElectricityCategory,
        DoublePublisher
    >();
    private final Map<ElectricityCategory, Double> currentBreakdown = new HashMap<ElectricityCategory, Double>();

    {
        allChannelCurrentsPublisher.set(new double[powerDistribution.getNumChannels()]);

        for (ElectricityCategory electricityCategory : ElectricityCategory.values()) {
            String name = Arrays.stream(electricityCategory.name().split("_"))
                .map(word -> word.substring(0, 1).toUpperCase() + word.substring(1).toLowerCase())
                .collect(Collectors.joining(" "));
            energyBreakdownPublishers.put(
                electricityCategory,
                energyBreakdownTable.getDoubleTopic(name + " (Wh)").publish()
            );
            energyBreakdown.put(electricityCategory, 0.0);
            currentBreakdownPublishers.put(
                electricityCategory,
                currentBreakdownTable.getDoubleTopic(name + " (A)").publish()
            );
            currentBreakdown.put(electricityCategory, 0.0);
        }
    }

    private static boolean running = false;

    public Telemetry() {
        start();
    }

    public void start() {
        if (running) {
            return;
        }
        running = true;

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
    }

    public void update() {
        if (!running) {
            return;
        }
        aggregateData();
        logData();
    }

    private void aggregateData() {
        double period = TimedRobot.kDefaultPeriod;

        double swerveDriveCurrent =
            powerDistribution.getCurrent(12) +
            powerDistribution.getCurrent(7) +
            powerDistribution.getCurrent(18) +
            powerDistribution.getCurrent(0);
        double swerveSteerCurrent =
            powerDistribution.getCurrent(11) +
            powerDistribution.getCurrent(8) +
            powerDistribution.getCurrent(19) +
            powerDistribution.getCurrent(1);
        double flywheelCurrent =
            powerDistribution.getCurrent(5) +
            powerDistribution.getCurrent(6) +
            powerDistribution.getCurrent(13) +
            powerDistribution.getCurrent(14);
        double hoodCurrent = powerDistribution.getCurrent(10);
        double neckCurrent = powerDistribution.getCurrent(9);
        double hopperCurrent = powerDistribution.getCurrent(16);
        double intakePivotCurrent = powerDistribution.getCurrent(4);
        double intakeRollerCurrent = powerDistribution.getCurrent(3);

        double powerDistributionVoltage = powerDistribution.getVoltage();

        incrementEnergyBreakdown(
            ElectricityCategory.SWERVE_DRIVE,
            period * powerDistributionVoltage * swerveDriveCurrent
        );
        incrementEnergyBreakdown(
            ElectricityCategory.SWERVE_STEER,
            period * powerDistributionVoltage * swerveSteerCurrent
        );
        incrementEnergyBreakdown(ElectricityCategory.FLYWHEEL, period * powerDistributionVoltage * flywheelCurrent);
        incrementEnergyBreakdown(ElectricityCategory.HOOD, period * powerDistributionVoltage * hoodCurrent);
        incrementEnergyBreakdown(ElectricityCategory.NECK, period * powerDistributionVoltage * neckCurrent);
        incrementEnergyBreakdown(ElectricityCategory.HOPPER, period * powerDistributionVoltage * hopperCurrent);
        incrementEnergyBreakdown(
            ElectricityCategory.INTAKE_PIVOT,
            period * powerDistributionVoltage * intakePivotCurrent
        );
        incrementEnergyBreakdown(
            ElectricityCategory.INTAKE_ROLLER,
            period * powerDistributionVoltage * intakeRollerCurrent
        );

        currentBreakdown.put(ElectricityCategory.SWERVE_DRIVE, swerveDriveCurrent);
        currentBreakdown.put(ElectricityCategory.SWERVE_STEER, swerveSteerCurrent);
        currentBreakdown.put(ElectricityCategory.FLYWHEEL, flywheelCurrent);
        currentBreakdown.put(ElectricityCategory.HOOD, hoodCurrent);
        currentBreakdown.put(ElectricityCategory.NECK, neckCurrent);
        currentBreakdown.put(ElectricityCategory.HOPPER, hopperCurrent);
        currentBreakdown.put(ElectricityCategory.INTAKE_PIVOT, intakePivotCurrent);
        currentBreakdown.put(ElectricityCategory.INTAKE_ROLLER, intakeRollerCurrent);
    }

    private void logData() {
        batteryVoltagePublisher.set(RobotController.getBatteryVoltage());

        totalEnergyPublisher.set(energyBreakdown.values().stream().mapToDouble(Double::doubleValue).sum());
        totalCurrentPublisher.set(powerDistribution.getTotalCurrent());
        allChannelCurrentsPublisher.set(powerDistribution.getAllCurrents());

        energyBreakdownPublishers.forEach((electricityCategory, publisher) -> {
            double energy = energyBreakdown.get(electricityCategory);
            publisher.set(energy);
        });
        currentBreakdownPublishers.forEach((electricityCategory, publisher) -> {
            double current = currentBreakdown.get(electricityCategory);
            publisher.set(current);
        });
    }

    private void incrementEnergyBreakdown(ElectricityCategory electricityCategory, double incrementJoules) {
        energyBreakdown.put(
            electricityCategory,
            energyBreakdown.get(electricityCategory) + incrementJoules * JOULES_TO_WATT_HOURS
        );
    }
}
