package frc.robot.operator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualXboxOperatorModality implements OperatorModality {

    private final CommandXboxController primaryXbox;
    private final CommandXboxController secondaryXbox;
    private final double triggerThreshold;

    public DualXboxOperatorModality(int primaryPort, int secondaryPort, double triggerThreshold) {
        this.primaryXbox = new CommandXboxController(primaryPort);
        this.secondaryXbox = new CommandXboxController(secondaryPort);
        this.triggerThreshold = triggerThreshold;
    }

    @Override
    public double translateX() {
        return -primaryXbox.getLeftY();
    }

    @Override
    public double translateY() {
        return -primaryXbox.getLeftX();
    }

    @Override
    public double rotation() {
        return -primaryXbox.getRightX();
    }

    @Override
    public Trigger zero() {
        return primaryXbox.rightTrigger(triggerThreshold);
    }

    @Override
    public Trigger slowMode() {
        return primaryXbox.leftTrigger(triggerThreshold);
    }

    @Override
    public Trigger driverAssist() {
        return primaryXbox.leftBumper();
    }

    @Override
    public Trigger intake() {
        return secondaryXbox.leftBumper();
    }

    @Override
    public Trigger outtake() {
        return secondaryXbox.leftTrigger(triggerThreshold);
    }

    @Override
    public Trigger retractIntake() {
        return secondaryXbox.y();
    }

    @Override
    public Trigger autoShoot() {
        return primaryXbox.rightBumper();
    }

    @Override
    public Trigger hubShoot() {
        return secondaryXbox.rightTrigger(triggerThreshold);
    }

    @Override
    public Trigger towerShoot() {
        return secondaryXbox.rightBumper();
    }

    @Override
    public Trigger autoLob() {
        return new Trigger(() -> false);
    }

    @Override
    public Trigger revShooter() {
        return secondaryXbox.povLeft();
    }

    public XboxController getPrimaryHID() {
        return primaryXbox.getHID();
    }

    public XboxController getSecondaryHID() {
        return secondaryXbox.getHID();
    }
}
