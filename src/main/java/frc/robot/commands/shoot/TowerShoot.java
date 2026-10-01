package frc.robot.commands.shoot;

import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;

public class TowerShoot extends ManualShoot {

    public TowerShoot(Shooter shooter, Intake intake, Hopper hopper) {
        super(ShooterSetpoint.FROM_TOWER, shooter, intake, hopper);
    }
}
