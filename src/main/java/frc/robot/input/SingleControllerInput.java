package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class SingleControllerInput implements MoInput {
    private static final double DEADZONE = 0.1;
    private static final double CURVE = 2.5;

    private final XboxController controller;

    public SingleControllerInput(Constants.HIDPort port) {
        this.controller = new XboxController(port.port);
    }

    @Override
    public double getForwardSpeedRequest() {
        return curve(deadzone(controller.getLeftY(), DEADZONE), CURVE);
    }

    @Override
    public double getLeftSpeedRequest() {
        return curve(deadzone(controller.getLeftX(), DEADZONE), CURVE);
    }

    @Override
    public double getTurnRequest() {
        return curve(deadzone(controller.getRightX(), DEADZONE), CURVE);
    }

}
