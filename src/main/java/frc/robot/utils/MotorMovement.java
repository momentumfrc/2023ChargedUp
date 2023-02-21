package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch.Type;

public final class MotorMovement {
    private MotorMovement() {
    }

    public static boolean isMovementLimited(CANSparkMax controller, double setpoint, Type limitType) {
        return
            (setpoint > 0 && controller.getForwardLimitSwitch(limitType).isPressed()) ||
            (setpoint < 0 && controller.getReverseLimitSwitch(limitType).isPressed());
    }

    public static boolean isOutOfBounds(CANSparkMax controller, Type limitType) {
        return
            controller.getForwardLimitSwitch(limitType).isPressed() ||
            controller.getReverseLimitSwitch(limitType).isPressed();
    }

    public static void moveLimited(CANSparkMax controller, double setpoint, Type limitType) {
        if (isMovementLimited(controller, setpoint, limitType)) {
            controller.stopMotor();
        } else {
            controller.set(setpoint);
        }
    }
}
