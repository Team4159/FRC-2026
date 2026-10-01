package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/** Applies the configured driver deadbands, response curves, and input limits. */
public final class DriverInputProcessor {

    private DriverInputProcessor() {}

    public static Translation2d translation(double x, double y, double deadband, double radius, double exponent) {
        Vector<N2> input = MathUtil.applyDeadband(VecBuilder.fill(x, y), deadband, 1.0).div(radius);
        if (input.norm() > 0.0) {
            input = input.unit().times(Math.pow(input.norm(), exponent));
        }
        if (input.norm() > 1.0) {
            input = input.div(input.norm());
        }
        return new Translation2d(input);
    }

    public static double rotation(double input, double deadband, double exponent) {
        double filtered = MathUtil.applyDeadband(Math.abs(input), deadband, 1.0);
        return Math.pow(filtered, exponent) * Math.signum(input);
    }
}
