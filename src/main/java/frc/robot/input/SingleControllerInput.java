package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
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

    @Override
    public double getDirectShoulderRequest() {
        return this.controller.getRightTriggerAxis() - this.controller.getLeftTriggerAxis();
    }

    @Override
    public double getDirectWristRequest() {
        return this.controller.getRightY();
    }
}
