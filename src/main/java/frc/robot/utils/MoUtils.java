package frc.robot.utils;

import com.revrobotics.RelativeEncoder;


public class MoUtils {
    private static final double ENCODER_ZERO_ZONE = 0.2;

    public static void setupRelativeEncoder(RelativeEncoder relEncoder, double absPos, double absZero, double ratio) {
        relEncoder.setPositionConversionFactor(1/ratio);
        relEncoder.setVelocityConversionFactor(1/ratio);

        double pos = absPos;
        pos = (pos + 1 - absZero) % 1;
        if(pos > (1 - ENCODER_ZERO_ZONE)) {
            pos -= 1;
        }
        relEncoder.setPosition(pos);
    }

    public static double rotToRad(double rot) {
        return 2 * Math.PI * (rot - 0.5);
    }

    public static double radToRot(double rad) {
        return (rad / (2 * Math.PI)) + 0.5;
    }
}
