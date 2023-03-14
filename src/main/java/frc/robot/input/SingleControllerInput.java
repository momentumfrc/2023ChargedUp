package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;
import frc.robot.utils.MoPrefs.Pref;

public class SingleControllerInput implements MoInput {
    private final XboxController controller;

    private Pref<Double> deadzone = MoPrefs.driveDeadzone;
    private Pref<Double> curve = MoPrefs.driveCurve;

    public SingleControllerInput(Constants.HIDPort port) {
        this.controller = new XboxController(port.port);
    }

    private double applyInputTransforms(double input) {
        return curve(deadzone(input, deadzone.get()), curve.get());
    }

    @Override
    public double getForwardSpeedRequest() {
        return applyInputTransforms(controller.getLeftY());
    }

    @Override
    public double getLeftSpeedRequest() {
        return -1 * applyInputTransforms(controller.getLeftX());
    }

    @Override
    public double getTurnRequest() {
        return -1 * applyInputTransforms(controller.getRightX());
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        return controller.getLeftStickButton();
    }

    private double getDirectShoulderRequest() {
        return applyInputTransforms(this.controller.getRightTriggerAxis() - this.controller.getLeftTriggerAxis());
    }

    private double getDirectWristRequest() {
        return applyInputTransforms(this.controller.getRightY());
    }

    @Override
    public ArmMovementRequest getDirectArmMovementRequest() {
        return new ArmMovementRequest(getDirectShoulderRequest(), getDirectWristRequest());
    }

    @Override
    public ArmPosition getCraneControlArmMovementRequest() {
        return new ArmPosition(this.controller.getRightTriggerAxis(), this.controller.getLeftTriggerAxis());
    }

    @Override
    public boolean getShouldIntake() {
        return this.controller.getLeftBumper();
    }

    @Override
    public boolean getShouldExhaust() {
        return this.controller.getRightBumper();
    }

    @Override
    public Optional<ArmSetpoint> getFineControlArmSetpoint() {
        double pov = this.controller.getPOV();
        boolean cubes = this.controller.getXButton();
        boolean cones = this.controller.getAButton();

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

        if(pov != -1) {
            return Optional.of(ArmSetpoint.STOW);
        }

        return Optional.empty();
    }

    @Override
    public boolean getSaveArmSetpoint() {
        return controller.getStartButton();
    }

    @Override
    public Optional<ArmSetpoint> getCraneControlArmSetpoint() {
        if (this.controller.getAButton()) {
            return Optional.of(ArmSetpoint.CONE_LOW);
        }
        if (this.controller.getXButton()) {
            return Optional.of(ArmSetpoint.CONE_MED);
        }
        if (this.controller.getYButton()) {
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
