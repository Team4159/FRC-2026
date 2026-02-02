package frc.lib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GraphMechanism {
    private final Mechanism2d mechanism = new Mechanism2d(0, 0);
    private final MechanismRoot2d root = mechanism.getRoot("root", 0, 0);
    private final ArrayList<MechanismLigament2d> ligaments = new ArrayList<>();
    private final double timeStep = 0.1;
    private final double lineWeight = 2;

    {
        if (RobotBase.isSimulation()) {
            MechanismLigament2d lastLigament = null;
            for (int i = 0; i < 25; i++) {
                var ligament = new MechanismLigament2d(String.valueOf(i), 0, 0);
                ligaments.add(ligament);
                ligament.setLineWeight(lineWeight);
                if (lastLigament == null) {
                    root.append(ligament);
                } else {
                    lastLigament.append(ligament);
                }
                lastLigament = ligament;
            }
        }
    }

    public GraphMechanism(String name) {
        SmartDashboard.putData(name, mechanism);
    }

    public void update(Function<double, Translation2d> graphFunction) {
        if (!RobotBase.isSimulation()) { return; }
        double lastAngle = 0;
        for (int i = 0; i < ligaments.size(); i++) {
            double t0 = timeStep * i;
            double t1 = timeStep * (i + 1);
            Translation2d p0 = graphFunction.apply(t0);
            Translation2d p1 = graphFunction.apply(t1);
            Translation2d delta = p1.minus(p0);
            double angle = delta.getAngle().getDegrees();
            var ligament = ligaments.get(i);
            ligament.setLength(delta.getNorm());
            ligament.setAngle(angle - lastAngle);
            lastAngle = angle;
        }
    }

    public void show() {
        for (MechanismLigament2d ligament : ligaments) {
            ligament.setLineWeight(lineWeight);
        }
    }

    public void hide() {
        for (MechanismLigament2d ligament : ligaments) {
            ligament.setLineWeight(0);
        }
    }
}
