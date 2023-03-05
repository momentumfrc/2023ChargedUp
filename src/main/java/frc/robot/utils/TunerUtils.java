package frc.robot.utils;

import com.momentum4999.utils.PIDTuner;
import com.momentum4999.utils.PIDTunerBuilder;
import com.playingwithfusion.CANVenom;
import frc.robot.Constants;

public class TunerUtils {
    public static PIDTuner forMoPIDF(MoPIDF controller, String controllerName) {
        return forMoPIDF(controller, controllerName, false);
    }
    public static PIDTuner forMoPIDF(MoPIDF controller, String controllerName, boolean hide) {
        return new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(controller::setP)
            .withI(controller::setI)
            .withD(controller::setD)
            .withFF(controller::setFF)
            .withIZone(controller::setIZone)
            .buildGraphValues()
            .withSetpointFrom(controller::getSetpoint)
            .withLastMeasurementFrom(controller::getLastMeasurement)
            .withLastOutputFrom(controller::getLastOutput)
            .finishGraphValues()
            .build();
    }

    public static PIDTuner forMoPID(MoPIDF controller, String controllerName) {
        return forMoPID(controller, controllerName, false);
    }
    public static PIDTuner forMoPID(MoPIDF controller, String controllerName, boolean hide) {
        return new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(controller::setP)
            .withI(controller::setI)
            .withD(controller::setD)
            .withIZone(controller::setIZone)
            .buildGraphValues()
            .withSetpointFrom(controller::getSetpoint)
            .withLastMeasurementFrom(controller::getLastMeasurement)
            .withLastOutputFrom(controller::getLastOutput)
            .finishGraphValues()
            .build();
    }

    public static PIDTuner forVenom(CANVenom venom, String controllerName) {
        return forVenom(venom, controllerName, false);
    }
    public static PIDTuner forVenom(CANVenom venom, String controllerName, boolean hide) {
        return new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(venom::setKP)
            .withI(venom::setKI)
            .withD(venom::setKD)
            .withFF(venom::setKF)
            .buildGraphValues()
            .withSetpointFrom(venom::getPIDTarget)
            .withLastMeasurementFrom(venom::getSpeed)
            .withLastOutputFrom(venom::get)
            .finishGraphValues()
            .build();
    }

    public static PIDTuner forMoSparkMax(MoSparkMaxPID sparkMax, String controllerName) {
        return forMoSparkMax(sparkMax, controllerName, false);
    }

    public static PIDTuner forMoSparkMax(MoSparkMaxPID sparkMax, String controllerName, boolean hide) {
        PIDTunerBuilder builder = new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(sparkMax::setP)
            .withI(sparkMax::setI)
            .withD(sparkMax::setD)
            .withFF(sparkMax::setFF);

        if(sparkMax.getType() == MoSparkMaxPID.Type.SMARTMOTION) {
            builder = builder
                .withProperty("maxVel", (v) -> sparkMax.getPID().setSmartMotionMaxVelocity(v, sparkMax.getPidSlot()))
                .withProperty("maxAccel", (a) -> sparkMax.getPID().setSmartMotionMaxAccel(a, sparkMax.getPidSlot()));
        }

        return builder
            .withGraphValues(sparkMax)
            .build();
    }
}
