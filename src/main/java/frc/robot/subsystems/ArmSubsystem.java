package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.input.MoInput;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.MoSparkMaxPID;
import frc.robot.utils.TunerUtils;
import frc.robot.utils.MoSparkMaxPID.Type;

public class ArmSubsystem extends SubsystemBase {
    private static final double SHOULDER_RATIO = 100 * 36 / 15;
    private static final double WRIST_RATIO = 60 * 24 / 14;
    public final SendableChooser<Command> armChooser = new SendableChooser<>();

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

    private final MoSparkMaxPID shoulderVelocityPID = new MoSparkMaxPID(Type.VELOCITY, leftShoulder);
    private final MoSparkMaxPID wristVelocityPID = new MoSparkMaxPID(Type.VELOCITY, wrist);

    private final TeleopArmCommand directArmControlCommand;
    private final TeleopArmCommand pidArmControlCommand;

    public ArmSubsystem(MoInput input) {
        leftShoulder.setInverted(false);
        rightShoulder.follow(leftShoulder, true);

        TunerUtils.forSparkMaxSmartMotion(shoulderVelocityPID, "Shoulder Vel. PID");
        TunerUtils.forSparkMaxSmartMotion(wristVelocityPID, "Wrist Vel. PID");

        this.directArmControlCommand = new TeleopArmCommand.Direct(this, input, false);
        this.pidArmControlCommand = new TeleopArmCommand.Direct(this, input, true);
        this.setDefaultCommand(directArmControlCommand);

        armChooser.setDefaultOption("Direct", directArmControlCommand);
        armChooser.addOption("Direct (PID)", pidArmControlCommand);

        MoShuffleboard.getInstance().matchTab.add("Arm Control Mode", armChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        MoPrefs.absShoulderZero.subscribe(ratio -> shoulderEncoder.setPositionConversionFactor(1/ratio), true);
        MoPrefs.absWristZero.subscribe(ratio -> wristEncoder.setPositionConversionFactor(1/ratio), true);

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

        this.zero();
        MoShuffleboard.getInstance().settingsTab
            .add("Recalculate Arm Position", new InstantCommand(this::zero));
    }

    @Override
    public void periodic() {
        this.updateDefaultCommand();
    }

    public void zero() {
        double shoulder = this.shoulderAbsEncoder.getPosition();
        double wrist = this.wristAbsEncoder.getPosition();

        shoulder = (shoulder + 1 - MoPrefs.absShoulderZero.get()) % 1;
        wrist = (wrist + 1 - MoPrefs.absWristZero.get()) % 1;

        this.shoulderEncoder.setPosition(shoulder);
        this.wristEncoder.setPosition(wrist);
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

    /**
     * @param target -1.0 to 1.0, each being the max speed in that direction
     */
    public void adjustShoulders(boolean pid, double target) {
        target = limitShoulderMovement(target);
        if (pid) {
            shoulderVelocityPID.setReference(target * MoPrefs.shoulderMaxRpm.get());
        } else {
            leftShoulder.set(target);
        }
    }

    /**
     * @param target -1.0 to 1.0, each being the max speed in that direction
     */
    public void adjustWrist(boolean pid, double target) {
        target = limitWristMovement(target);

        if (pid) {
            wristVelocityPID.setReference(target * MoPrefs.wristMaxRpm.get());
        } else {
            wrist.set(target);
        }
    }

    public void updateDefaultCommand() {
        if (armChooser.getSelected() != null && armChooser.getSelected() != this.getDefaultCommand()) {
            this.setDefaultCommand(armChooser.getSelected());
        }
    }
}
