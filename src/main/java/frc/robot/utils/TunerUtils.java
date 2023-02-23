package frc.robot.utils;

import java.util.function.DoubleSupplier;

import com.momentum4999.utils.PIDTuner;
import com.momentum4999.utils.PIDTunerBuilder;
import com.momentum4999.utils.PIDTuner.PIDGraphValues;
import com.playingwithfusion.CANVenom;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

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

    public static PIDTuner forSparkMaxSmartMotion(MoSparkMaxPID sparkMax, String controllerName) {
        return forSparkMaxSmartMotion(sparkMax, controllerName, false);
    }

    public static PIDTuner forSparkMaxSmartMotion(MoSparkMaxPID sparkMax, String controllerName, boolean hide) {
        return new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(sparkMax.getPID()::setP)
            .withI(sparkMax.getPID()::setI)
            .withD(sparkMax.getPID()::setD)
            .withFF(sparkMax.getPID()::setFF)
            .withProperty("maxVel", (v) -> sparkMax.getPID().setSmartMotionMaxVelocity(v, 0))
            .withProperty("maxAccel", (a) -> sparkMax.getPID().setSmartMotionMaxAccel(a, 0))
            .buildGraphValues()
            .withSetpointFrom(sparkMax::getReference)
            .withLastMeasurementFrom(sparkMax::getMeasurement)
            .withLastOutputFrom(sparkMax::getOutput)
            .finishGraphValues()
            .build();
    }
}
