package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.TunerUtils;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_LEFT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_RIGHT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(
        Constants.ARM_WRIST.address, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MoPIDF leftShoulderVelocityPID = new MoPIDF();
    private final MoPIDF rightShoulderVelocityPID = new MoPIDF();
    private final MoPIDF wristVelocityPID = new MoPIDF();

    private final MoPIDF shoulderPositionPID = new MoPIDF();
    private final MoPIDF wristPositionPID = new MoPIDF();

    public ArmSubsystem() {
        rightShoulder.setInverted(true); // TODO: Verify

        TunerUtils.forSparkMaxSmartMotion(leftShoulder, "Left Shoulder Vel. PID");
        TunerUtils.forMoPID(rightShoulderVelocityPID, "Right Shoulder Vel. PID");
        TunerUtils.forMoPID(wristVelocityPID, "Wrist Vel. PID");

        TunerUtils.forMoPID(shoulderPositionPID, "Shoulder Pos. PID");
        TunerUtils.forMoPID(wristPositionPID, "Wrist Pos. PID");
    }

    @Override
    public void periodic() {
    }

    public void adjustShoulders(double power) {
        power *= MoPrefs.shoulderSetpointRpm.get();

        leftShoulder.set(leftShoulderVelocityPID.calculate(leftShoulder.getEncoder().getVelocity(), power));
        rightShoulder.set(rightShoulderVelocityPID.calculate(rightShoulder.getEncoder().getVelocity(), power));
    }

    public void adjustWrist(double power) {
        power *= MoPrefs.wristSetpointRpm.get();

        wrist.set(wristVelocityPID.calculate(wrist.getEncoder().getVelocity(), power));
    }
}
