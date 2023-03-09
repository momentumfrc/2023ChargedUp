package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import java.util.Map;

import com.momentum4999.utils.Utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.input.MoInput;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.MoSparkMaxPID;
import frc.robot.utils.TunerUtils;
import frc.robot.utils.MoSparkMaxPID.Type;

public class ArmSubsystem extends SubsystemBase {
    public enum ArmControlMode {
        FALLBACK_DIRECT_POWER,
        DIRECT_VELOCITY,
        SMART_MOTION
    };

    public final SendableChooser<ArmControlMode> armChooser = MoShuffleboard.enumToChooser(ArmControlMode.class);

    private final CANSparkMax leftShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_LEFT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_RIGHT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(
        Constants.ARM_WRIST.address, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final RelativeEncoder shoulderEncoder = leftShoulder.getEncoder();
    private final SparkMaxAbsoluteEncoder shoulderAbsEncoder = leftShoulder.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final RelativeEncoder wristEncoder = wrist.getEncoder();
    private final SparkMaxAbsoluteEncoder wristAbsEncoder = wrist.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final MoSparkMaxPID shoulderVelocityPID = new MoSparkMaxPID(Type.VELOCITY, leftShoulder, 0);
    private final MoSparkMaxPID wristVelocityPID = new MoSparkMaxPID(Type.VELOCITY, wrist, 0);
    private final MoSparkMaxPID shoulderSmartMotionPID = new MoSparkMaxPID(Type.SMARTMOTION, leftShoulder, 1);
    private final MoSparkMaxPID wristSmartMotionPID = new MoSparkMaxPID(Type.SMARTMOTION, wrist, 1);

    public ArmSubsystem(MoInput input) {
        leftShoulder.setInverted(false);
        rightShoulder.follow(leftShoulder, true);

        TunerUtils.forMoSparkMax(shoulderVelocityPID, "Shoulder Vel. PID", false);
        TunerUtils.forMoSparkMax(wristVelocityPID, "Wrist Vel. PID", false);
        TunerUtils.forMoSparkMax(shoulderSmartMotionPID, "Shoulder Pos. PID", false);
        TunerUtils.forMoSparkMax(wristSmartMotionPID, "Wrist Pos. PID", false);

        MoShuffleboard.getInstance().settingsTab.add("Arm Control Mode", armChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        MoPrefs.shoulderEncoderRatio.subscribe(ratio -> {
            shoulderEncoder.setPositionConversionFactor(1/ratio);
            shoulderEncoder.setVelocityConversionFactor(1/ratio);
        }, true);
        MoPrefs.wristEncoderRatio.subscribe(ratio -> {
            wristEncoder.setPositionConversionFactor(1/ratio);
            wristEncoder.setVelocityConversionFactor(1/ratio);
        }, true);

        var shoulderGroup = MoShuffleboard.getInstance().matchTab
            .getLayout("Shoulder Position", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "RIGHT"));
        shoulderGroup.addDouble("Relative", shoulderEncoder::getPosition);
        shoulderGroup.addDouble("Absolute", shoulderAbsEncoder::getPosition);

        var wristGroup = MoShuffleboard.getInstance().matchTab
            .getLayout("Wrist Position", BuiltInLayouts.kList)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "RIGHT"));
        wristGroup.addDouble("Relative", wristEncoder::getPosition);
        wristGroup.addDouble("Absolute", wristAbsEncoder::getPosition);

        MoPrefs.absShoulderZero.subscribe(zero -> {
            double shoulder = this.shoulderAbsEncoder.getPosition();
            shoulder = (shoulder + 1 - zero) % 1;
            if(shoulder > 0.5) {
                shoulder -= 1;
            }
            this.shoulderEncoder.setPosition(shoulder);
        }, true);

        MoPrefs.absWristZero.subscribe(zero -> {
            double wrist = this.wristAbsEncoder.getPosition();
            wrist = (wrist + 1 - zero) % 1;
            if(wrist > 0.5) {
                wrist -= 1;
            }
            this.wristEncoder.setPosition(wrist);
        }, true);
    }

    private double limitShoulderMovement(double target) {
        if(target > 0) {
            if(this.shoulderEncoder.getPosition() > MoPrefs.shoulderMaxRevolutions.get()) {
                return 0;
            }
        } else {
            if(this.shoulderEncoder.getPosition() < 0) {
                return 0;
            }
        }
        return target;
    }

    private double limitWristMovement(double target) {
        if(target > 0) {
            if(this.wristEncoder.getPosition() > MoPrefs.wristMaxRevolutions.get()) {
                return 0;
            }
        } else {
            if(this.wristEncoder.getPosition() < 0) {
                return 0;
            }
        }
        return target;
    }

    public void adjustDirectPower(double shoulderPower, double wristPower) {
        shoulderPower = limitShoulderMovement(shoulderPower);
        wristPower = limitWristMovement(wristPower);
        leftShoulder.set(shoulderPower);
        wrist.set(wristPower);
    }

    public void adjustVelocity(double shoulderVelocity, double wristVelocity) {
        shoulderVelocity = limitShoulderMovement(shoulderVelocity);
        wristVelocity = limitWristMovement(wristVelocity);
        shoulderVelocityPID.setReference(shoulderVelocity);
        wristVelocityPID.setReference(wristVelocity);
    }

    public void adjustSmartPosition(double shoulderPosition, double wristPosition) {
        shoulderPosition = Utils.clip(shoulderPosition, 0, MoPrefs.shoulderMaxRevolutions.get());
        wristPosition = Utils.clip(wristPosition, 0, MoPrefs.wristMaxRevolutions.get());

        shoulderSmartMotionPID.setReference(shoulderPosition);
        wristSmartMotionPID.setReference(wristPosition);
    }
}
