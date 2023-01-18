package frc.robot.utils;

import com.momentum4999.utils.PIDTuner.PIDAdapter;

import edu.wpi.first.math.controller.PIDController;

public class MoPIDF implements PIDAdapter {
    private PIDController pid = new PIDController(0, 0, 0);
    private double kF = 0;

    private double setpoint = 0;
    private double lastMeasurement = 0;

    public double calculate(double measurement, double setpoint) {
        this.setpoint = setpoint;
        this.lastMeasurement = measurement;
        return pid.calculate(measurement, setpoint) + (kF * setpoint);
    }

    @Override
    public void setP(double kP) {
        pid.setP(kP);
    }

    @Override
    public void setI(double kI) {
        pid.setI(kI);
    }

    @Override
    public void setD(double kD) {
        pid.setD(kD);
    }

    @Override
    public void setFF(double kFF) {
        kF = kFF;
    }

    @Override
    public void setIZone(double kIZone) {
        pid.setIntegratorRange(-kIZone, kIZone);
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public double getCurrentValue() {
        return lastMeasurement;
    }


}
