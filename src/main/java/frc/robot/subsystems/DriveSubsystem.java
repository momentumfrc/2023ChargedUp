package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final CANVenom frontLeftMtr = new CANVenom(Constants.DRIVE_LEFT_FRONT.address);
    private final CANVenom frontRightMtr = new CANVenom(Constants.DRIVE_RIGHT_FRONT.address);
    private final CANVenom rearLeftMtr = new CANVenom(Constants.DRIVE_LEFT_REAR.address);
    private final CANVenom rearRightMtr = new CANVenom(Constants.DRIVE_RIGHT_REAR.address);

    private final MecanumDrive drive = new MecanumDrive(frontLeftMtr, rearLeftMtr, frontRightMtr, rearRightMtr);

    private final AHRS gyro;

    public DriveSubsystem(AHRS gyro) {
        this.gyro = gyro;
    }

    public void driveCartesianFieldOriented(double fwdRequest, double leftRequest, double turnRequest) {
        // TODO: get the Rotation2d from the odometry, not the gyro (so that it uses the AprilTags)
        //       Example: this.odometry.getPoseMeters().getRotation();
        drive.driveCartesian(fwdRequest, leftRequest, turnRequest, gyro.getRotation2d());
    }

    public void driveCartesianRobotOriented(double fwdRequest, double leftRequest, double turnRequest) {
        drive.driveCartesian(fwdRequest, leftRequest, turnRequest);
    }

}
