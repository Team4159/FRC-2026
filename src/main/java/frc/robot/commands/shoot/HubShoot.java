package frc.robot.commands.shoot;

import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;

public class HubShoot extends ManualShoot {

    public HubShoot(Shooter shooter, Intake intake, Hopper hopper) {
        super(ShooterSetpoint.FROM_HUB, shooter, intake, hopper);
    }
}
