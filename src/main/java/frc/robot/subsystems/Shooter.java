package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter {
    private TalonFX l_topMotor = new TalonFX(ShooterConstants.lTopMotor_ControllerPort);
    private TalonFX l_botMotor = new TalonFX(ShooterConstants.lBotMotor_ControllerPort);
    private TalonFX r_topMotor = new TalonFX(ShooterConstants.rTopMotor_ControllerPort);
    private TalonFX r_botMotor = new TalonFX(ShooterConstants.rBotMotor_ControllerPort);
    private TalonFX hoodMotor = new TalonFX(ShooterConstants.hoodMotor_ControllerPort);

    private final PositionVoltage hoodPositionVoltage = new PositionVoltage(0);
    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);

    Slot0Configs hoodConfig = new Slot0Configs();
    {
        hoodConfig.kP = Constants.ShooterConstants.kP;
        hoodConfig.kI = Constants.ShooterConstants.kI;
        hoodConfig.kD = Constants.ShooterConstants.kD;
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
    public void adjustHood() {
        hoodMotor.setControl(hoodPositionVoltage.withPosition(0));
    }

}
