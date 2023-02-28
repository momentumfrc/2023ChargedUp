package frc.robot.utils;

import com.momentum4999.utils.PIDTuner.PIDGraphValues;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class MoSparkMaxPID implements PIDGraphValues {
    private final Type type;
    private final CANSparkMax controller;
    private double lastReference;

    public MoSparkMaxPID(Type type, CANSparkMax controller) {
        this.type = type;
        this.controller = controller;
    }

    public SparkMaxPIDController getPID() {
        return this.controller.getPIDController();
    }

    @Override
    public double getLastOutput() {
        return this.controller.get();
    }

    @Override
    public double getSetpoint() {
        return this.lastReference;
    }

    @Override
    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
                return this.controller.getEncoder().getPosition();
            case VELOCITY:
                return this.controller.getEncoder().getVelocity();
        }

        return 0;
    }

    public void setReference(double value) {
        this.getPID().setReference(value, this.type == Type.VELOCITY ? ControlType.kVelocity : ControlType.kPosition);
        this.lastReference = value;
    }

    public enum Type {
        POSITION, VELOCITY;
    }
}
