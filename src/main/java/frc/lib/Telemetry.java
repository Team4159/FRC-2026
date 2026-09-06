package frc.lib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.DrivetrainConstants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Telemetry {

    private static enum EnergyCategory {
        SWERVE_DRIVE,
        SWERVE_STEER,
        FLYWHEEL,
        HOOD,
        NECK,
        HOPPER,
        INTAKE_PIVOT,
        INTAKE_ROLLER,
    }

    private static final double SWERVE_MODULE_SPREAD = 0.25;

    private static final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    private static final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    private static final NetworkTable energyTable = networkTableInstance.getTable("Energy Usage");
    private static final DoublePublisher energyUsedPublisher = energyTable
        .getDoubleTopic("Energy Used Total")
        .publish();
    private static final NetworkTable energyBreakdownTable = energyTable.getSubTable("Energy Usage Breakdown");
    private static final Map<EnergyCategory, DoublePublisher> energyBreakdownPublishers = new HashMap<
        EnergyCategory,
        DoublePublisher
    >();
    private static final Map<EnergyCategory, Energy> energyBreakdown = new HashMap<EnergyCategory, Energy>();

    static {
        for (EnergyCategory energyCategory : EnergyCategory.values()) {
            String name = Arrays.stream(energyCategory.name().split("_"))
                .map(word -> word.substring(0, 1).toUpperCase() + word.substring(1).toLowerCase())
                .collect(Collectors.joining(" "));
            energyBreakdownPublishers.put(energyCategory, energyBreakdownTable.getDoubleTopic(name).publish());
            energyBreakdown.put(energyCategory, Joules.of(0.0));
        }
    }

    private static final NetworkTable subsystemTable = networkTableInstance.getTable("Subsystems");
    private static final NetworkTable drivetrainTable = subsystemTable.getSubTable("Drivetrain");
    private static final StructPublisher<Pose2d> drivetrainPosePublisher = drivetrainTable
        .getStructTopic("Pose", Pose2d.struct)
        .publish();
    private static final Mechanism2d[] swerveModuleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    private static final MechanismLigament2d[] swerveModuleVelocityTargets = new MechanismLigament2d[4];
    private static final MechanismLigament2d[] swerveModuleVelocityStates = new MechanismLigament2d[4];
    private static final MechanismLigament2d[] swerveModuleAngleTargets = new MechanismLigament2d[4];
    private static final MechanismLigament2d[] swerveModuleAngleStates = new MechanismLigament2d[4];

    static {
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

    private static SwerveDriveState lastDrivetrainState;

    public static void start() {
        if (running) {
            return;
        }
        running = true;

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        SignalLogger.setPath(DataLogManager.getLogDir());
        SignalLogger.start();

        if (RobotBase.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start();
    }

    public static void telemetrizeDrivetrain(SwerveDriveState drivetrainState) {
        lastDrivetrainState = drivetrainState;

        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, drivetrainState.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, drivetrainState.Speeds);
        SignalLogger.writeStructArray(
            "DriveState/ModuleStates",
            SwerveModuleState.struct,
            drivetrainState.ModuleStates
        );
        SignalLogger.writeStructArray(
            "DriveState/ModuleTargets",
            SwerveModuleState.struct,
            drivetrainState.ModuleTargets
        );
        SignalLogger.writeStructArray(
            "DriveState/ModulePositions",
            SwerveModulePosition.struct,
            drivetrainState.ModulePositions
        );
        SignalLogger.writeDouble("DriveState/OdometryPeriod", drivetrainState.OdometryPeriod, "seconds");
    }

    public static void run() {
        aggregateData();
        logData();
    }

    private static void aggregateData() {
        Time period = Milliseconds.of(20.0);

        Voltage powerDistributionVoltage = Volts.of(powerDistribution.getVoltage());
        // TODO: find channels
        incrementEnergyCategory(
            EnergyCategory.SWERVE_DRIVE,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.SWERVE_STEER,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.FLYWHEEL,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.HOOD,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.NECK,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.HOPPER,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.INTAKE_PIVOT,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
        incrementEnergyCategory(
            EnergyCategory.INTAKE_ROLLER,
            powerDistributionVoltage.times(Amps.of(powerDistribution.getCurrent(0))).times(period)
        );
    }

    private static void logData() {
        energyUsedPublisher.set(energyBreakdown.values().stream().mapToDouble(Energy::baseUnitMagnitude).sum());
        energyBreakdownPublishers.forEach((energyCategory, publisher) -> {
            double energy = energyBreakdown.get(energyCategory).baseUnitMagnitude();
            publisher.set(energy);
        });

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

    private static void incrementEnergyCategory(EnergyCategory energyCategory, Energy increment) {
        energyBreakdown.put(energyCategory, energyBreakdown.get(energyCategory).plus(increment));
    }

    private static void populateLigaments(
        MechanismLigament2d[] ligamentsTarget,
        Mechanism2d[] mechanisms,
        Vector<N2>[] rootPositions,
        String rootName,
        String ligamentName,
        double length,
        double lineWidth,
        Color8Bit color
    ) {
        if (ligamentsTarget.length != mechanisms.length || mechanisms.length != rootPositions.length) {
            throw new IllegalArgumentException("Array lengths must be equal");
        }
        for (int i = 0; i < ligamentsTarget.length; i++) {
            ligamentsTarget[i] = mechanisms[i]
                .getRoot(rootName, rootPositions[i].get(0), rootPositions[i].get(1))
                .append(new MechanismLigament2d(ligamentName, length, 0, lineWidth, color));
        }
    }

    private Telemetry() {}
}
