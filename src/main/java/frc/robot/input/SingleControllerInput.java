package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import com.momentum4999.utils.controllers.LogitechF310;

import frc.robot.Constants;

public class SingleControllerInput implements MoInput {
    private static final double DEADZONE = 0.1;
    private static final double CURVE = 2.5;

    private final LogitechF310 controller;

    public SingleControllerInput(Constants.HIDPort port) {
        this.controller = new LogitechF310(port.port);
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
