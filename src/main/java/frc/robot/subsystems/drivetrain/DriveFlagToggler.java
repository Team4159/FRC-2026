package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.operator.OperatorConstants.DriveFlag;

public class DriveFlagToggler extends Command {

    private final Drivetrain drivetrain;
    private final DriveFlag driveFlag;

    public DriveFlagToggler(Drivetrain drivetrain, DriveFlag driveFlag) {
        this.drivetrain = drivetrain;
        this.driveFlag = driveFlag;
    }

    @Override
    public void initialize() {
        drivetrain.getDriveFlags().setValue(driveFlag, !drivetrain.getDriveFlags().getDefaultValue(driveFlag));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getDriveFlags().setValue(driveFlag, drivetrain.getDriveFlags().getDefaultValue(driveFlag));
    }
}
