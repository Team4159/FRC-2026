package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;
import frc.robot.subsystems.shooter.FeederConstants.FeederSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;

public class ManualShoot extends Shoot {

    private final ShooterSetpoint shooterSetpoint;

    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    private final Timer timer = new Timer();
    private boolean feedFlag = false;

    public ManualShoot(ShooterSetpoint shooterSetpoint, Shooter shooter, Intake intake, Hopper hopper) {
        if (shooterSetpoint.pitch.isEmpty()) {
            throw new IllegalArgumentException("Shooter setpoint must have non-empty pitch");
        }
        this.shooterSetpoint = shooterSetpoint;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFlywheelMotorVelocity(shooterSetpoint);
        shooter.setHoodPitchComplement(shooterSetpoint);
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());
        timer.reset();
        feedFlag = false;
    }

    @Override
    public void execute() {
        // if (!timer.hasElapsed(ShooterConstants.backwardsTime)) {
        //     //run neck backwards if at the beginning
        //     shooter.setFeederDutyCycle(FeederSetpoint.UNJAM);
        //     hopper.setDutyCycle(FeederSetpoint.STOP);
        // }
        if (shooter.isAtHoodPitch() && shooter.isAtFlywheelVelocity()) {
            feedFlag = true;
        }
        if (feedFlag) {
            //shoot the fuel if at the right pitch
            shooter.setFeederDutyCycle(FeederSetpoint.FEED);
            hopper.setDutyCycle(HopperSetpoint.FEED);
        } else {
            //otherwise just wait
            shooter.setFeederDutyCycle(FeederSetpoint.STOP);
            hopper.setDutyCycle(HopperSetpoint.STOP);
        }

        SmartDashboard.putBoolean("isAtPitch", shooter.isAtHoodPitch());
        SmartDashboard.putBoolean("isAtVelocity", shooter.isAtFlywheelVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.restHood();
        shooter.setFeederDutyCycle(FeederSetpoint.STOP);
        hopper.setDutyCycle(HopperSetpoint.STOP);
        CommandScheduler.getInstance().schedule(intake.new ChangeSetpoints(IntakeSetpoint.BOUNCE_UP));
    }
}
