package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MotorMovement;

public class ArmSubsystem extends SubsystemBase {
    public static final Type SHOULDER_LIMIT = Type.kNormallyOpen;
    public static final Type WRIST_LIMIT = Type.kNormallyOpen;

    private final CANSparkMax leftShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_LEFT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_RIGHT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(
        Constants.ARM_WRIST.address, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ArmSubsystem() {
        rightShoulder.setInverted(true); // TODO: Verify
    }

    @Override
    public void periodic() {
        if (MotorMovement.isOutOfBounds(leftShoulder, SHOULDER_LIMIT)) {
            leftShoulder.stopMotor();
        }
        if (MotorMovement.isOutOfBounds(rightShoulder, SHOULDER_LIMIT)) {
            rightShoulder.stopMotor();
        }
        if (MotorMovement.isOutOfBounds(wrist, WRIST_LIMIT)) {
            wrist.stopMotor();
        }
    }

    public void adjustArm(double power) {
        power *= MoPrefs.shoulderSetpointRpm.get();

        MotorMovement.moveLimited(leftShoulder, power, SHOULDER_LIMIT);
        MotorMovement.moveLimited(rightShoulder, power, SHOULDER_LIMIT);
    }

    public void adjustWrist(double power) {
        power *= MoPrefs.wristSetpointRpm.get();

        MotorMovement.moveLimited(wrist, power, WRIST_LIMIT);
    }

    public void idleArm() {
        leftShoulder.stopMotor();
        rightShoulder.stopMotor();
    }

    public void idleWrist() {
        wrist.stopMotor();
    }
}
