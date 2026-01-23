package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;

public class Shooter extends SubsystemBase{
    private TalonFX l_topMotor;
    private TalonFX l_botMotor;
    private TalonFX r_topMotor;
    private TalonFX r_botMotor;
    private TalonFX hoodMotor;

    private final PositionVoltage hoodPositionVoltage;
    private final VelocityVoltage shooterVelocityVoltage;

    public Shooter() {
        Slot0Configs hoodConfig = new Slot0Configs();
        hoodConfig.kP = Constants.ShooterConstants.SkP;
        hoodConfig.kI = Constants.ShooterConstants.SkI;
        hoodConfig.kD = Constants.ShooterConstants.SkD;

        Slot0Configs shooterConfig = new Slot0Configs();
        shooterConfig.kP = Constants.ShooterConstants.SkP;
        shooterConfig.kI = Constants.ShooterConstants.SkI;
        shooterConfig.kD = Constants.ShooterConstants.SkD;

        hoodMotor = new TalonFX(Constants.HoodConstants.HoodId);
        l_topMotor = new TalonFX(Constants.ShooterConstants.ShooterIDOne);
        l_botMotor = new TalonFX(Constants.ShooterConstants.ShooterIDTwo);
        r_topMotor = new TalonFX(Constants.ShooterConstants.ShooterIDThree);
        r_botMotor = new TalonFX(Constants.ShooterConstants.ShooterIDFour);
        
        l_topMotor.getConfigurator().apply(shooterConfig);
        l_botMotor.getConfigurator().apply(shooterConfig);
        r_topMotor.getConfigurator().apply(shooterConfig);
        r_botMotor.getConfigurator().apply(shooterConfig);
        
        hoodMotor.getConfigurator().apply(hoodConfig);

        hoodPositionVoltage = new PositionVoltage(0);

        shooterVelocityVoltage = new VelocityVoltage(0);
    }

    // set speed
    public void setSpeed(double speed) {
        shooterVelocityVoltage.withVelocity(speed);
        l_topMotor.setControl(shooterVelocityVoltage);
        l_botMotor.setControl(shooterVelocityVoltage);
        r_topMotor.setControl(shooterVelocityVoltage);
        r_botMotor.setControl(shooterVelocityVoltage); 
    }

    // adjust hood
    public void adjustHood(double v, double h, double d) { // check below for more info
        // MEASUED IN FEET
        //double v = 40; // initial velocity
        //double h = 7; // height
        //double d = 15; // distance
        double g = 32.2; // gravity
        double fC = (g * Math.pow(d, 2)) / (2 * Math.pow(v, 2)); // first constant
        double sC = fC + h; // second constant
        double r = Math.atan((d + Math.sqrt(Math.pow(d, 2) - (4 * fC) * sC)) / (2 * fC)); // radians
        double angle = r * 180/Math.PI;


        hoodMotor.setControl(hoodPositionVoltage.withPosition(angle));
        
    }

    public void stopHood() {
        hoodMotor.setControl(hoodPositionVoltage.withPosition(0));
    }

}
