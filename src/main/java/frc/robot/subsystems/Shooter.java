package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    private TalonFX hood;
    private TalonFX shooterMotorOne;
    private TalonFX shooterMotorTwo;
    private TalonFX shooterMotorThree;
    private TalonFX shooterMotorFour;

    private PositionVoltage hoodPositionVoltage;
    private PositionVoltage motorOneVoltage;
    private PositionVoltage motorTwoVoltage;
    private PositionVoltage motorThreeVoltage;
    private PositionVoltage motorFourVoltage;

    public Shooter() {
        Slot0Configs shooterConfig = new Slot0Configs();
        shooterConfig.kP = Constants.ShooterConstants.SkP;
        shooterConfig.kI = Constants.ShooterConstants.SkI;
        shooterConfig.kD = Constants.ShooterConstants.SkD;

        Slot0Configs hoodConfig = new Slot0Configs();
        hoodConfig.kP = Constants.HoodConstants.kP;
        hoodConfig.kI = Constants.HoodConstants.kI;
        hoodConfig.kD = Constants.HoodConstants.kD;

        hood = new TalonFX(Constants.HoodConstants.HoodId);
        shooterMotorOne = new TalonFX(Constants.ShooterConstants.ShooterIDOne);
        shooterMotorTwo = new TalonFX(Constants.ShooterConstants.ShooterIDTwo);
        shooterMotorThree = new TalonFX(Constants.ShooterConstants.ShooterIDThree);
        shooterMotorFour = new TalonFX(Constants.ShooterConstants.ShooterIDFour);

        shooterMotorOne.getConfigurator().apply(shooterConfig);
        shooterMotorTwo.getConfigurator().apply(shooterConfig);
        shooterMotorThree.getConfigurator().apply(shooterConfig);
        shooterMotorFour.getConfigurator().apply(shooterConfig);

        hood.getConfigurator().apply(hoodConfig);

        motorOneVoltage = new PositionVoltage(0);
        motorTwoVoltage = new PositionVoltage(0);
        motorThreeVoltage = new PositionVoltage(0);
        motorFourVoltage = new PositionVoltage(0);

        hoodPositionVoltage = new PositionVoltage(0);
    }

    public void setHoodAngle(double position) {
        hood.setControl(hoodPositionVoltage.withPosition(position));
    }

    public void periodic() {

    }
}
