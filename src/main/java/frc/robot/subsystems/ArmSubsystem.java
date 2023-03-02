package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    public final SendableChooser<Command> armChooser = new SendableChooser<>();

    private final CANSparkMax leftShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_LEFT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightShoulder = new CANSparkMax(
        Constants.ARM_SHOULDER_RIGHT.address, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wrist = new CANSparkMax(
        Constants.ARM_WRIST.address, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MoSparkMaxPID shoulderVelocityPID = new MoSparkMaxPID(Type.VELOCITY, leftShoulder);
    private final MoSparkMaxPID wristVelocityPID = new MoSparkMaxPID(Type.VELOCITY, wrist);

    private final TeleopArmCommand directArmControlCommand;
    private final TeleopArmCommand pidArmControlCommand;

    public ArmSubsystem(MoInput input) {
        rightShoulder.setInverted(true); // TODO: Verify
        rightShoulder.follow(leftShoulder);

        TunerUtils.forSparkMaxSmartMotion(shoulderVelocityPID, "Shoulder Vel. PID");
        TunerUtils.forSparkMaxSmartMotion(wristVelocityPID, "Wrist Vel. PID");

        this.directArmControlCommand = new TeleopArmCommand.Direct(this, input, false);
        this.pidArmControlCommand = new TeleopArmCommand.Direct(this, input, true);
        this.setDefaultCommand(directArmControlCommand);

        armChooser.addOption("Direct", directArmControlCommand);
        armChooser.addOption("Direct (PID)", pidArmControlCommand);

        MoShuffleboard.getInstance().matchTab.add("Arm Control Mode", armChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
 
        this.zero();
        MoShuffleboard.getInstance().settingsTab
            .add("Recalculate Arm Position", new InstantCommand(this::zero));
    }

    @Override
    public void periodic() {
        this.updateDefaultCommand();
    }

    public void zero() {
        this.leftShoulder.getEncoder().setPosition(
            ((this.leftShoulder.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .getPosition() - MoPrefs.absShoulderZero.get() + 1) % 1) * MoPrefs.shoulderAbsToRel.get()
        );
        this.wrist.getEncoder().setPosition(
            ((this.wrist.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .getPosition() - MoPrefs.absWristZero.get() + 1) % 1) * MoPrefs.wristAbsToRel.get()
        );
    }

    private boolean shoulderMoveLimited(double movement) {
        double pos = this.leftShoulder.getEncoder().getPosition();
        if (movement > 0) {
            return pos >= MoPrefs.relShoulderMax.get();
        }
        if (movement < 0) {
            return pos <= 0;
        }
        return false;
    }

    private boolean wristMoveLimited(double movement) {
        double pos = this.wrist.getEncoder().getPosition();
        if (movement > 0) {
            return pos >= MoPrefs.relWristMax.get();
        }
        if (movement < 0) {
            return pos <= 0;
        }
        return false;
    }

    /**
     * @param target -1.0 to 1.0, each being the max speed in that direction
     */
    public void adjustShoulders(boolean pid, double target) {
        if (shoulderMoveLimited(target)) {
            leftShoulder.stopMotor();
        }

        if (pid) {
            shoulderVelocityPID.setReference(target * MoPrefs.shoulderSetpointRpm.get());
        } else {
            leftShoulder.set(target);
        }
    }

    /**
     * @param target -1.0 to 1.0, each being the max speed in that direction
     */
    public void adjustWrist(boolean pid, double target) {
        if (wristMoveLimited(target)) {
            wrist.stopMotor();
        }

        if (pid) {
            wristVelocityPID.setReference(target * MoPrefs.wristSetpointRpm.get());
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
