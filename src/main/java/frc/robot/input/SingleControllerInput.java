package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import com.momentum4999.utils.controllers.LogitechF310;

public class SingleControllerInput implements MoInput {
    private static final double DEADZONE = 0.1;
    private static final double CURVE = 2.5;

    private final LogitechF310 controller;

    public SingleControllerInput(LogitechF310 controller) {
        this.controller = controller;
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
