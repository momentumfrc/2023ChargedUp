package frc.robot.input;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;
import frc.robot.utils.MoPrefs.Pref;

import static com.momentum4999.utils.Utils.*;

public class DualControllerInput implements MoInput {

    private final XboxController driveController;
    private final XboxController armController;

    private Pref<Double> driveDeadzone = MoPrefs.driveDeadzone;
    private Pref<Double> driveCurve = MoPrefs.driveCurve;

    public DualControllerInput(Constants.HIDPort drivePort, Constants.HIDPort armPort) {
        this.driveController = new XboxController(drivePort.port);
        this.armController = new XboxController(armPort.port);
    }

    private double applyDriveInputTransforms(double value) {
        return curve(deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    private double applyArmInputTransforms(double value) {
        // TODO: determine if we'd rather have a separate deadzone/curve for the arm controller
        return curve(deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    @Override
    public double getForwardSpeedRequest() {
        return applyDriveInputTransforms(driveController.getLeftY());
    }

    @Override
    public double getLeftSpeedRequest() {
        return applyDriveInputTransforms(driveController.getLeftX());
    }

    @Override
    public double getTurnRequest() {
        return applyDriveInputTransforms(driveController.getRightX());
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        return driveController.getRightBumper();
    }

    @Override
    public boolean getShouldIntake() {
        return armController.getLeftBumper();
    }

    @Override
    public boolean getShouldExhaust() {
        return armController.getRightBumper();
    }

    @Override
    public ArmMovementRequest getDirectArmMovementRequest() {
        return new ArmMovementRequest(
            applyArmInputTransforms(armController.getLeftY()),
            applyArmInputTransforms(armController.getRightY())
        );
    }

    @Override
    public Optional<ArmSetpoint> getFineControlArmSetpoint() {
        double pov = armController.getPOV();
        boolean cubes = armController.getXButton();
        boolean cones = armController.getAButton();
        boolean stow = armController.getBButton();

        if(cubes) {
            if(pov == 0) {
                return Optional.of(ArmSetpoint.CUBE_HIGH);
            } else if(pov == 90) {
                return Optional.of(ArmSetpoint.CUBE_PICKUP);
            } else if(pov == 180) {
                return Optional.of(ArmSetpoint.CUBE_LOW);
            } else if(pov == 270) {
                return Optional.of(ArmSetpoint.CUBE_MED);
            }
        }

        if(cones) {
            if(pov == 0) {
                return Optional.of(ArmSetpoint.CONE_HIGH);
            } else if(pov == 90) {
                return Optional.of(ArmSetpoint.CONE_PICKUP);
            } else if(pov == 180) {
                return Optional.of(ArmSetpoint.CONE_LOW);
            } else if(pov == 270) {
                return Optional.of(ArmSetpoint.CONE_MED);
            }
        }

        if(stow) {
            return Optional.of(ArmSetpoint.STOW);
        }

        return Optional.empty();
    }

    @Override
    public boolean getSaveArmSetpoint() {
        return armController.getStartButton();
    }

    @Override
    public ArmPosition getCraneControlArmMovementRequest() {
        return new ArmPosition(this.armController.getLeftTriggerAxis(), this.armController.getRightTriggerAxis());
    }

    @Override
    public Optional<ArmSetpoint> getCraneControlArmSetpoint() {
        // Duplicate of single controller, could be improved to take advantage of the whole controller
        if (this.armController.getAButton()) {
            return Optional.of(ArmSetpoint.CONE_LOW);
        }
        if (this.armController.getXButton()) {
            return Optional.of(ArmSetpoint.CONE_MED);
        }
        if (this.armController.getYButton()) {
            return Optional.of(ArmSetpoint.CONE_HIGH);
        }
        if (this.getShouldIntake()) {
            return Optional.of(ArmSetpoint.CONE_PICKUP);
        }
        if (this.getShouldExhaust()) {
            return Optional.of(ArmSetpoint.CUBE_PICKUP);
        }
        return Optional.empty();
    }
}
