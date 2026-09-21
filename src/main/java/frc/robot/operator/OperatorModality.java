package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorModality {
    double driveX();

    double driveY();

    double rotation();

    Trigger zero();

    Trigger slowMode();

    Trigger driverAssist();

    Trigger intake();

    Trigger outtake();

    Trigger retractIntake();

    Trigger autoShoot();

    Trigger hubShoot();

    Trigger towerShoot();

    Trigger autoLob();
}
