package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {

    // TODO: tune stddev values
    public static final Matrix<N3, N1> SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(1, 1, 4);
    public static final Matrix<N3, N1> MULTI_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.25, 0.25, 1);

    public static final Transform3d LEFT_SHOOTER_CAMERA_TRANSFORM = new Transform3d(
        Units.inchesToMeters(-1.2887),
        Units.inchesToMeters(8.8466),
        Units.inchesToMeters(21.1190),
        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-5))
    );
    public static final Transform3d RIGHT_SHOOTER_CAMERA_TRANSFORM = new Transform3d(
        Units.inchesToMeters(-1.2887),
        Units.inchesToMeters(-8.8466),
        Units.inchesToMeters(21.1190),
        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(5))
    );

    public static final double AMBIGUITY_THRESHOLD = 0.15;
}
