package frc.lib;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
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

    private final double SWERVE_MODULE_SPREAD = 0.25;
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

    private final NetworkTable subsystemTable = networkTableInstance.getTable("Subsystems");
    private final NetworkTable drivetrainTable = subsystemTable.getSubTable("Drivetrain");
    private final StructPublisher<Pose2d> drivetrainPosePublisher = drivetrainTable
        .getStructTopic("Pose", Pose2d.struct)
        .publish();
    private final Mechanism2d[] swerveModuleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    private final MechanismLigament2d[] swerveModuleVelocityTargets = new MechanismLigament2d[4];
    private final MechanismLigament2d[] swerveModuleVelocityStates = new MechanismLigament2d[4];
    private final MechanismLigament2d[] swerveModuleAngleTargets = new MechanismLigament2d[4];
    private final MechanismLigament2d[] swerveModuleAngleStates = new MechanismLigament2d[4];

    {
        @SuppressWarnings("unchecked")
        Vector<N2>[] rootPositions = new Vector[] {
            VecBuilder.fill(0.5 - SWERVE_MODULE_SPREAD, 0.5 - SWERVE_MODULE_SPREAD),
            VecBuilder.fill(0.5 - SWERVE_MODULE_SPREAD, 0.5 + SWERVE_MODULE_SPREAD),
            VecBuilder.fill(0.5 + SWERVE_MODULE_SPREAD, 0.5 - SWERVE_MODULE_SPREAD),
            VecBuilder.fill(0.5 + SWERVE_MODULE_SPREAD, 0.5 + SWERVE_MODULE_SPREAD),
        };
        populateLigaments(
            swerveModuleVelocityTargets,
            swerveModuleMechanisms,
            rootPositions,
            "4_VelocityTargetRoot",
            "VelocityTarget",
            0.0,
            8.0,
            new Color8Bit(71, 125, 54)
        );
        populateLigaments(
            swerveModuleVelocityStates,
            swerveModuleMechanisms,
            rootPositions,
            "3_VelocityStateRoot",
            "VelocityState",
            0.0,
            10.0,
            new Color8Bit(235, 137, 52)
        );
        populateLigaments(
            swerveModuleAngleTargets,
            swerveModuleMechanisms,
            rootPositions,
            "2_AngleTargetRoot",
            "AngleTarget",
            0.1,
            2.0,
            new Color8Bit(109, 224, 73)
        );
        populateLigaments(
            swerveModuleAngleStates,
            swerveModuleMechanisms,
            rootPositions,
            "1_AngleStateRoot",
            "AngleState",
            0.1,
            4.0,
            new Color8Bit(255, 255, 255)
        );

        for (int i = 0; i < swerveModuleMechanisms.length; i++) {
            NetworkTable table = drivetrainTable.getSubTable("Swerve Module " + i);
            SendableBuilderImpl builder = new SendableBuilderImpl();
            builder.setTable(table);
            builder.startListeners();
            swerveModuleMechanisms[i].initSendable(builder);
        }
    }

    private static boolean running = false;

    private SwerveDriveState lastDrivetrainState;

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

        SignalLogger.setPath(DataLogManager.getLogDir());
        SignalLogger.start();
    }

    public void update() {
        if (!running) {
            return;
        }
        aggregateData();
        logData();
    }

    public void telemetrizeDrivetrain(SwerveDriveState drivetrainState) {
        lastDrivetrainState = drivetrainState;

        // SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, drivetrainState.Pose);
        // SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, drivetrainState.Speeds);
        // SignalLogger.writeStructArray(
        //     "DriveState/ModuleStates",
        //     SwerveModuleState.struct,
        //     drivetrainState.ModuleStates
        // );
        // SignalLogger.writeStructArray(
        //     "DriveState/ModuleTargets",
        //     SwerveModuleState.struct,
        //     drivetrainState.ModuleTargets
        // );
        // SignalLogger.writeStructArray(
        //     "DriveState/ModulePositions",
        //     SwerveModulePosition.struct,
        //     drivetrainState.ModulePositions
        // );
        // SignalLogger.writeDouble("DriveState/OdometryPeriod", drivetrainState.OdometryPeriod, "seconds");
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

        if (lastDrivetrainState != null) {
            drivetrainPosePublisher.set(lastDrivetrainState.Pose);
            for (int i = 0; i < swerveModuleMechanisms.length; i++) {
                SwerveModuleState state = lastDrivetrainState.ModuleStates[i];
                swerveModuleVelocityStates[i].setAngle(state.angle);
                swerveModuleVelocityStates[i].setLength(
                    (state.speedMetersPerSecond / DrivetrainConstants.MAX_TRANSLATION_SPEED) * SWERVE_MODULE_SPREAD
                );
                swerveModuleAngleStates[i].setAngle(state.angle);
                SwerveModuleState target = lastDrivetrainState.ModuleTargets[i];
                swerveModuleVelocityTargets[i].setAngle(target.angle);
                swerveModuleVelocityTargets[i].setLength(
                    (target.speedMetersPerSecond / DrivetrainConstants.MAX_TRANSLATION_SPEED) * SWERVE_MODULE_SPREAD
                );
                swerveModuleAngleTargets[i].setAngle(target.angle);
            }
        }
    }

    private void incrementEnergyBreakdown(ElectricityCategory electricityCategory, double incrementJoules) {
        energyBreakdown.put(
            electricityCategory,
            energyBreakdown.get(electricityCategory) + incrementJoules * JOULES_TO_WATT_HOURS
        );
    }

    private void populateLigaments(
        MechanismLigament2d[] ligamentsTarget,
        Mechanism2d[] mechanisms,
        Vector<N2>[] rootPositions,
        String rootName,
        String ligamentName,
        double length,
        double lineWidth,
        Color8Bit color
    ) {
        if (mechanisms.length != ligamentsTarget.length || mechanisms.length != rootPositions.length) {
            throw new IllegalArgumentException("Array lengths must be equal");
        }
        for (int i = 0; i < ligamentsTarget.length; i++) {
            ligamentsTarget[i] = mechanisms[i]
                .getRoot(rootName, rootPositions[i].get(0), rootPositions[i].get(1))
                .append(new MechanismLigament2d(ligamentName, length, 0, lineWidth, color));
        }
    }
}
