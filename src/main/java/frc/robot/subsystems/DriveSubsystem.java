package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoShuffleboard;

public class DriveSubsystem extends SubsystemBase {
    private final CANVenom frontLeftMtr = new CANVenom(Constants.DRIVE_LEFT_FRONT.address);
    private final CANVenom frontRightMtr = new CANVenom(Constants.DRIVE_RIGHT_FRONT.address);
    private final CANVenom rearLeftMtr = new CANVenom(Constants.DRIVE_LEFT_REAR.address);
    private final CANVenom rearRightMtr = new CANVenom(Constants.DRIVE_RIGHT_REAR.address);

    private final MecanumDrive drive = new MecanumDrive(frontLeftMtr, rearLeftMtr, frontRightMtr, rearRightMtr);

    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private GenericSubscriber shouldDriveFieldOriented = MoShuffleboard.getInstance().settingsTab
        .add("Field-Oriented Drive", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest) {
        if(shouldDriveFieldOriented.getBoolean(true)) {
            driveCartesianFieldOriented(fwdRequest, leftRequest, turnRequest);
        } else {
            driveCartesianRobotOriented(fwdRequest, leftRequest, turnRequest);
        }
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
