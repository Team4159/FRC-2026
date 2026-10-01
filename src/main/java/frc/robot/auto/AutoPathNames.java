package frc.robot.auto;

/** Pure helpers for assembling the Choreo trajectory names used by configurable autos. */
public final class AutoPathNames {

    private AutoPathNames() {}

    public static String outpostStartToIntake(String direction) {
        return direction + "StartToMROutpostIntake";
    }

    public static String outpostIntakeToShoot() {
        return "MROutpostIntakeToMRShoot";
    }

    public static String middleStartToShoot(String direction) {
        return direction + "StartToShoot";
    }

    public static String startToIntake(String intake) {
        return "StartTo" + intake;
    }

    public static String intakeToShoot(String intake, String shoot) {
        return intake + "To" + shoot;
    }

    public static String shootToIntake(String shoot, String intake) {
        return shoot + "To" + intake;
    }
}
