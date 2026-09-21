package frc.robot.operator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleXboxOperatorModality implements OperatorModality {

    private final CommandXboxController xbox;
    private final double triggerThreshold;

    private final Trigger autoShootBase, autoShoot;
    private final Trigger hubShootBase, hubShoot;
    private final Trigger towerShoot;

    public SingleXboxOperatorModality(int port, double triggerThreshold) {
        this.xbox = new CommandXboxController(port);
        this.triggerThreshold = triggerThreshold;
        autoShootBase = xbox.rightTrigger(triggerThreshold);
        hubShootBase = xbox.rightBumper();
        towerShoot = autoShootBase.and(hubShootBase);
        autoShoot = autoShootBase.and(towerShoot.negate());
        hubShoot = hubShootBase.and(towerShoot.negate());
    }

    @Override
    public double driveX() {
        return -xbox.getLeftY();
    }

    @Override
    public double driveY() {
        return -xbox.getLeftX();
    }

    @Override
    public double rotation() {
        return -xbox.getRightX();
    }

    @Override
    public Trigger zero() {
        return xbox.back();
    }

    @Override
    public Trigger slowMode() {
        return xbox.leftTrigger(triggerThreshold);
    }

    @Override
    public Trigger driverAssist() {
        return xbox.a();
    }

    @Override
    public Trigger intake() {
        return xbox.leftBumper();
    }

    @Override
    public Trigger outtake() {
        return xbox.x();
    }

    @Override
    public Trigger retractIntake() {
        return xbox.b();
    }

    @Override
    public Trigger autoShoot() {
        return autoShoot;
    }

    @Override
    public Trigger hubShoot() {
        return hubShoot;
    }

    @Override
    public Trigger towerShoot() {
        return towerShoot;
    }

    @Override
    public Trigger autoLob() {
        return autoShoot;
    }

    public XboxController getHID() {
        return xbox.getHID();
    }
}
