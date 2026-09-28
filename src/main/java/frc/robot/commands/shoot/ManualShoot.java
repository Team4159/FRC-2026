package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
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
        //     shooter.setFeederDutyCycle(FeederState.UNJAM);
        //     hopper.setDutyCycle(HopperState.STOP);
        // }
        if (shooter.isAtHoodPitch() && shooter.isAtFlywheelVelocity()) {
            feedFlag = true;
        }
        if (feedFlag) {
            //shoot the fuel if at the right pitch
            shooter.setFeederDutyCycle(FeederState.FEED);
            hopper.setDutyCycle(HopperState.FEED);
        } else {
            //otherwise just wait
            shooter.setFeederDutyCycle(FeederState.STOP);
            hopper.setDutyCycle(HopperState.STOP);
        }

        SmartDashboard.putBoolean("isAtPitch", shooter.isAtHoodPitch());
        SmartDashboard.putBoolean("isAtVelocity", shooter.isAtFlywheelVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.restHood();
        shooter.setFeederDutyCycle(FeederState.STOP);
        hopper.setDutyCycle(HopperState.STOP);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.BOUNCE_UP));
    }
}
