package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

public class MoPIDF extends PIDController {
    public MoPIDF() {
        super(0, 0, 0);
    }

    private double kF = 0;

    private double setpoint = 0;
    private double lastMeasurement = 0;
    private double lastOutput = 0;

    public double calculate(double measurement, double setpoint) {
        this.setpoint = setpoint;
        this.lastMeasurement = measurement;
        lastOutput = super.calculate(measurement, setpoint) + (kF * setpoint);
        return lastOutput;
    }

    public void setFF(double kFF) {
        kF = kFF;
    }

    public void setIZone(double kIZone) {
        super.setIntegratorRange(-kIZone, kIZone);
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getLastMeasurement() {
        return lastMeasurement;
    }

    public double getLastOutput() {
        return lastOutput;
    }
}
