package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoSparkMaxPID;
import frc.robot.utils.TunerUtils;
import frc.robot.utils.MoSparkMaxPID.Type;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_LEFT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_RIGHT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(
        Constants.ARM_WRIST.address, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MoSparkMaxPID leftShoulderVelocityPID = new MoSparkMaxPID(Type.VELOCITY, leftShoulder);
    private final MoSparkMaxPID rightShoulderVelocityPID = new MoSparkMaxPID(Type.VELOCITY, rightShoulder);
    private final MoSparkMaxPID wristVelocityPID = new MoSparkMaxPID(Type.VELOCITY, wrist);

    public ArmSubsystem() {
        rightShoulder.setInverted(true); // TODO: Verify

        TunerUtils.forSparkMaxSmartMotion(leftShoulderVelocityPID, "Left Shoulder Vel. PID");
        TunerUtils.forSparkMaxSmartMotion(rightShoulderVelocityPID, "Right Shoulder Vel. PID");
        TunerUtils.forSparkMaxSmartMotion(wristVelocityPID, "Wrist Vel. PID");
    }

    @Override
    public void periodic() {
    }

    public void adjustShoulders(double power) {
        power *= MoPrefs.shoulderSetpointRpm.get();

        leftShoulderVelocityPID.setReference(power);
        rightShoulderVelocityPID.setReference(power);
    }

    public void adjustWrist(double power) {
        power *= MoPrefs.wristSetpointRpm.get();

        wristVelocityPID.setReference(power);
    }
}
