package frc.robot.utils;

import com.momentum4999.utils.PIDTuner;
import com.momentum4999.utils.PIDTuner.PIDAdapter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class MoPID extends PIDController implements PIDAdapter {
    private double lastMeasurement = 0;
    private PIDTuner tuner;

    public MoPID(String name) {
        super(0, 0, 0);
        tuner = new PIDTuner(name, this, Constants.TUNER_SETTINGS);
    }

    @Override
    public void setFF(double kFF) {
        // not supported on wpilib PIDControllers
        if(kFF != 0)
            DriverStation.reportWarning("Cannot set kFF on a wpilib PIDController", false);
    }

    @Override
    public void setIZone(double kIZone) {
        setIntegratorRange(-kIZone, kIZone);
    }

    @Override
    public double getCurrentValue() {
        return lastMeasurement;
    }

    @Override
    public double calculate(double measurement) {
        lastMeasurement = measurement;
        return super.calculate(measurement);
    }

}
