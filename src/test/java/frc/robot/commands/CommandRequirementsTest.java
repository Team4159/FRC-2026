package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shoot.AutoLob;
import frc.robot.commands.shoot.AutoShoot;
import frc.robot.commands.shoot.HubShoot;
import frc.robot.commands.shoot.ManualShoot;
import frc.robot.commands.shoot.TowerShoot;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorConstants.DriveMode;
import frc.robot.operator.OperatorModality;
import frc.robot.subsystems.drivetrain.DriveFlagToggler;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;
import frc.robot.subsystems.shooter.FeederConstants.FeederSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;
import java.util.Set;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class CommandRequirementsTest {

    private static final Trigger NEVER = new Trigger(() -> false);
    private static Drivetrain drivetrain;
    private static Intake intake;
    private static Hopper hopper;
    private static Shooter shooter;

    @BeforeAll
    static void createSubsystems() {
        assertTrue(HAL.initialize(500, 0));
        drivetrain = new Drivetrain(new TestOperatorModality());
        intake = new Intake();
        hopper = new Hopper();
        shooter = new Shooter();
    }

    @Test
    void drivetrainCommandsMatchDeclaredRequirements() {
        assertRequirements(drivetrain.createDriveCommand(DriveMode.TELEOP), drivetrain);
        assertRequirements(drivetrain.createDriveCommand(DriveMode.BRAKE), drivetrain);
        assertRequirements(drivetrain.createDriveCommand(DriveMode.POINT), drivetrain);
        assertRequirements(drivetrain.createDriveCommand(DriveMode.IDLE), drivetrain);
        assertRequirements(new DriveFlagToggler(drivetrain, DriveFlag.SLOW_MODE));
    }

    @Test
    void subsystemCommandsMatchDeclaredRequirements() {
        assertRequirements(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_ON), intake);
        assertRequirements(intake.new CompressIntake(), intake);
        assertRequirements(intake.new BounceIntake(), intake);
        assertRequirements(hopper.new ChangeSetpoint(HopperSetpoint.FEED));
        assertRequirements(shooter.new ChangeVelocity(RPM.of(1000)), shooter);
        assertRequirements(shooter.new ChangeFeederSetpoint(FeederSetpoint.FEED));
    }

    @Test
    void manualShootingCommandsReserveShooter() {
        assertRequirements(new ManualShoot(ShooterSetpoint.FROM_HUB, shooter, intake, hopper), shooter);
        assertRequirements(new HubShoot(shooter, intake, hopper), shooter);
        assertRequirements(new TowerShoot(shooter, intake, hopper), shooter);
    }

    @Test
    void autoShootAndLobReserveSharedSubsystemsOnlyInDriverControlledMode() {
        assertRequirements(new AutoShoot(drivetrain, shooter, hopper, intake, true), drivetrain, shooter, hopper);
        assertRequirements(new AutoLob(drivetrain, shooter, hopper, intake, true), drivetrain, shooter, hopper);
        assertRequirements(new AutoShoot(drivetrain, shooter, hopper, intake, false));
        assertRequirements(new AutoLob(drivetrain, shooter, hopper, intake, false));
    }

    private static void assertRequirements(Command command, Subsystem... expected) {
        assertEquals(Set.of(expected), command.getRequirements(), command.getName());
    }

    private static class TestOperatorModality implements OperatorModality {
        @Override
        public double translateX() { return 0.0; }
        @Override
        public double translateY() { return 0.0; }
        @Override
        public double rotation() { return 0.0; }
        @Override
        public Trigger zero() { return NEVER; }
        @Override
        public Trigger slowMode() { return NEVER; }
        @Override
        public Trigger driverAssist() { return NEVER; }
        @Override
        public Trigger intake() { return NEVER; }
        @Override
        public Trigger outtake() { return NEVER; }
        @Override
        public Trigger retractIntake() { return NEVER; }
        @Override
        public Trigger autoShoot() { return NEVER; }
        @Override
        public Trigger hubShoot() { return NEVER; }
        @Override
        public Trigger towerShoot() { return NEVER; }
        @Override
        public Trigger autoLob() { return NEVER; }
        @Override
        public Trigger revShooter() { return NEVER; }
    }
}
