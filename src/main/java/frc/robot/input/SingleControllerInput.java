package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.Pref;

public class SingleControllerInput implements MoInput {
    private final XboxController controller;

    private Pref<Double> deadzone = MoPrefs.driveDeadzone;
    private Pref<Double> curve = MoPrefs.driveCurve;

    private JoystickButton raiseArm;
    private JoystickButton lowerArm;
    private JoystickButton retractWrist;
    private JoystickButton extendWrist;

    public SingleControllerInput(Constants.HIDPort port) {
        this.controller = new XboxController(port.port);
        this.raiseArm = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        this.lowerArm = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        this.retractWrist = new JoystickButton(controller, XboxController.Button.kX.value);
        this.extendWrist = new JoystickButton(controller, XboxController.Button.kY.value);
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
    public boolean getShouldMaintainWristParallel() {
        return controller.getAButton();
    }

    @Override
    public JoystickButton getRaiseArmButton() {
        return this.raiseArm;
    }

    @Override
    public JoystickButton getLowerArmButton() {
        return this.lowerArm;
    }

    @Override
    public JoystickButton getRetractWristButton() {
        return this.retractWrist;
    }

    @Override
    public JoystickButton getExtendWristButton() {
        return this.extendWrist;
    }
}
