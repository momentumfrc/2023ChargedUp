package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

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
import frc.robot.utils.MoUtils;
import frc.robot.utils.TunerUtils;
import frc.robot.utils.MoSparkMaxPID.Type;

public class ArmSubsystem extends SubsystemBase {
    private static final double ENCODER_MAX_DRIFT = 0.5;

    public enum ArmControlMode {
        FALLBACK_DIRECT_POWER,
        DIRECT_VELOCITY,
        SMART_MOTION
    };

    public static class ArmPosition {
        public final double shoulderRotations;
        public final double wristRotations;

        public ArmPosition(double shoulderRotations, double wristRotations) {
            this.shoulderRotations = shoulderRotations;
            this.wristRotations = wristRotations;
        }

        @Override
        public boolean equals(Object other) {
            if(other instanceof ArmPosition) {
                ArmPosition posOth = (ArmPosition) other;
                return posOth.shoulderRotations == shoulderRotations
                    && posOth.wristRotations == wristRotations;
            }
            return false;
        }

        @Override
        public String toString() {
            return String.format("ArmPosition(shoulder=%.2f, wrist=%.2f)", shoulderRotations, wristRotations);
        }
    }

    public static class ArmMovementRequest {
        public final double shoulderPower;
        public final double wristPower;
        public final double shoulderVelocity;
        public final double wristVelocity;

        public ArmMovementRequest(double shoulder, double wrist) {
            this.shoulderPower = shoulder;
            this.wristPower = wrist;
            this.shoulderVelocity = shoulder * MoPrefs.shoulderMaxRpm.get();
            this.wristVelocity = wrist * MoPrefs.wristMaxRpm.get();
        }

        public boolean isZero() {
            return shoulderPower == 0 && wristPower == 0;
        }

        @Override
        public String toString() {
            return String.format("ArmMovementRequest(shoulderPower=%.2f, wristPower=%.2f)", shoulderPower, wristPower);
        }
    }

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

    public ArmSubsystem() {
        leftShoulder.restoreFactoryDefaults();
        rightShoulder.restoreFactoryDefaults();
        wrist.restoreFactoryDefaults();

        wristAbsEncoder.setInverted(true);

        leftShoulder.setInverted(false);
        rightShoulder.follow(leftShoulder, true);

        TunerUtils.forMoSparkMax(shoulderVelocityPID, "Shoulder Vel. PID", false);
        TunerUtils.forMoSparkMax(wristVelocityPID, "Wrist Vel. PID", false);
        TunerUtils.forMoSparkMax(shoulderSmartMotionPID, "Shoulder Pos. PID", false);
        TunerUtils.forMoSparkMax(wristSmartMotionPID, "Wrist Pos. PID", false);

        MoShuffleboard.getInstance().settingsTab.add("Arm Control Mode", armChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        MoPrefs.shoulderEncoderRatio.subscribe(ratio ->
            MoUtils.setupRelativeEncoder(shoulderEncoder, shoulderAbsEncoder.getPosition(), MoPrefs.absShoulderZero.get(), ratio)
        );
        MoPrefs.wristEncoderRatio.subscribe(ratio ->
            MoUtils.setupRelativeEncoder(wristEncoder, wristAbsEncoder.getPosition(), MoPrefs.absWristZero.get(), ratio)
        );

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

        MoPrefs.absShoulderZero.subscribe(zero ->
            MoUtils.setupRelativeEncoder(shoulderEncoder, shoulderAbsEncoder.getPosition(), zero, MoPrefs.shoulderEncoderRatio.get())
        );

        MoPrefs.absWristZero.subscribe(zero ->
            MoUtils.setupRelativeEncoder(wristEncoder, wristAbsEncoder.getPosition(), zero, MoPrefs.wristEncoderRatio.get())
        );

        var currentGroup = MoShuffleboard.getInstance().matchTab
            .getLayout("Arm Currents", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "RIGHT"));

        currentGroup.addDouble("Left Shoulder", leftShoulder::getOutputCurrent);
        currentGroup.addDouble("Right Shoulder", rightShoulder::getOutputCurrent);
        currentGroup.addDouble("Wrist", wrist::getOutputCurrent);

        MoPrefs.shoulderCurrentLimit.subscribe(limit -> {
            leftShoulder.setSmartCurrentLimit(limit.intValue());
            rightShoulder.setSmartCurrentLimit(limit.intValue());
        }, true);
        MoPrefs.wristCurrentLimit.subscribe(wrist::setSecondaryCurrentLimit, true);

        leftShoulder.setSoftLimit(SoftLimitDirection.kReverse, 0);
        wrist.setSoftLimit(SoftLimitDirection.kReverse, 0);

        MoPrefs.shoulderMaxRevolutions.subscribe(limit -> leftShoulder.setSoftLimit(SoftLimitDirection.kForward, limit.floatValue()), true);
        MoPrefs.wristMaxRevolutions.subscribe(limit -> wrist.setSoftLimit(SoftLimitDirection.kForward, limit.floatValue()), true);

        MoUtils.setupRelativeEncoder(shoulderEncoder, shoulderAbsEncoder.getPosition(), MoPrefs.absShoulderZero.get(), MoPrefs.shoulderEncoderRatio.get());
        MoUtils.setupRelativeEncoder(wristEncoder, wristAbsEncoder.getPosition(), MoPrefs.absWristZero.get(), MoPrefs.wristEncoderRatio.get());
    }

    public ArmPosition getPosition() {
        return new ArmPosition(
            shoulderEncoder.getPosition(),
            wristEncoder.getPosition()
        );
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

    private ArmMovementRequest limitArmMovement(ArmMovementRequest request) {
        return new ArmMovementRequest(
            limitShoulderMovement(request.shoulderPower),
            limitWristMovement(request.wristPower)
        );
    }

    private ArmPosition limitArmPosition(ArmPosition position) {
        return new ArmPosition(
            Utils.clip(position.shoulderRotations, 0, MoPrefs.shoulderMaxRevolutions.get()),
            Utils.clip(position.wristRotations, 0, MoPrefs.wristMaxRevolutions.get())
        );
    }

    public void adjustDirectPower(ArmMovementRequest movement) {
        movement = limitArmMovement(movement);
        leftShoulder.set(movement.shoulderPower);
        wrist.set(movement.wristPower);
    }


    public void adjustVelocity(ArmMovementRequest movement) {
        movement = limitArmMovement(movement);
        shoulderVelocityPID.setReference(movement.shoulderVelocity);
        wristVelocityPID.setReference(movement.wristVelocity);
    }

    public void adjustSmartPosition(ArmPosition position) {
        position = limitArmPosition(position);
        shoulderSmartMotionPID.setReference(position.shoulderRotations);
        wristSmartMotionPID.setReference(position.wristRotations);
    }

    public void stop() {
        leftShoulder.set(0);
        wrist.set(0);
    }

    public void reZero() {
        MoUtils.setupRelativeEncoder(shoulderEncoder, shoulderAbsEncoder.getPosition(), MoPrefs.absShoulderZero.get(), MoPrefs.shoulderEncoderRatio.get());
        MoUtils.setupRelativeEncoder(wristEncoder, wristAbsEncoder.getPosition(), MoPrefs.absWristZero.get(), MoPrefs.wristEncoderRatio.get());
    }

    @Override
    public void periodic() {
        if(Math.abs(shoulderEncoder.getPosition() - shoulderAbsEncoder.getPosition()) > ENCODER_MAX_DRIFT) {
            MoUtils.setupRelativeEncoder(shoulderEncoder, shoulderAbsEncoder.getPosition(), MoPrefs.absShoulderZero.get(), MoPrefs.shoulderEncoderRatio.get());
        }
        if(Math.abs(wristEncoder.getPosition() - wristAbsEncoder.getPosition()) > ENCODER_MAX_DRIFT) {
            MoUtils.setupRelativeEncoder(wristEncoder, wristAbsEncoder.getPosition(), MoPrefs.absWristZero.get(), MoPrefs.wristEncoderRatio.get());
        }
    }
}
