package frc.robot.utils;

import com.momentum4999.motune.PIDTuner;
import com.playingwithfusion.CANVenom;

import frc.robot.Constants;

public class TunerUtils {
    public static PIDTuner forMoPIDF(MoPIDF controller, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(controller::setP)
            .withI(controller::setI)
            .withD(controller::setD)
            .withFF(controller::setFF)
            .withIZone(controller::setIZone)
            .withSetpoint(controller::getSetpoint)
            .withMeasurement(controller::getLastMeasurement)
            .safeBuild();
    }

    public static PIDTuner forMoPID(MoPIDF controller, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(controller::setP)
            .withI(controller::setI)
            .withD(controller::setD)
            .withIZone(controller::setIZone)
            .withSetpoint(controller::getSetpoint)
            .withMeasurement(controller::getLastMeasurement)
            .safeBuild();
    }

    public static PIDTuner forVenom(CANVenom venom, String controllerName) {
        return PIDTuner.builder(controllerName)
            .withDataStoreFile(Constants.DATA_STORE_FILE)
            .withP(venom::setKP)
            .withI(venom::setKI)
            .withD(venom::setKD)
            .withFF(venom::setKF)
            .withSetpoint(venom::getPIDTarget)
            .withMeasurement(venom::getSpeed)
            .safeBuild();
    }
}
