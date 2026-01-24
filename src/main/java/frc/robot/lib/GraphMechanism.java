package frc.robot.lib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GraphMechanism {

    @FunctionalInterface
    public interface GraphFunction {
        Translation2d get(double t);
    }

    private final Mechanism2d mechanism = new Mechanism2d(0, 0);
    private final MechanismRoot2d root = mechanism.getRoot("root", 0, 0);
    private final ArrayList<MechanismLigament2d> ligaments = new ArrayList<>();
    private double timeStep = 0.1;
    {
        MechanismLigament2d lastLigament = null;
        for (int i = 0; i < 25; i++) {
            var ligament = new MechanismLigament2d(String.valueOf(i), 0, 0);
            ligaments.add(ligament);
            ligament.setLineWeight(2);
            if (lastLigament == null) {
                root.append(ligament);
            } else {
                lastLigament.append(ligament);
            }
            lastLigament = ligament;
        }
    }

    public GraphMechanism(String name) {
        SmartDashboard.putData(name, mechanism);
    }

    public void update(GraphFunction graphFunction) {
        double lastAngle = 0;
        for (int i = 0; i < ligaments.size(); i++) {
            double t0 = timeStep * i;
            double t1 = timeStep * (i + 1);
            Translation2d p0 = graphFunction.get(t0);
            Translation2d p1 = graphFunction.get(t1);
            Translation2d delta = p1.minus(p0);
            double angle = Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));

            var ligament = ligaments.get(i);
            ligament.setLength(Math.hypot(delta.getX(), delta.getY()));
            ligament.setAngle(angle - lastAngle);
            lastAngle = angle;
        }
    }
}
