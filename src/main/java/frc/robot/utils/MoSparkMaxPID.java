package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class MoSparkMaxPID {
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

    public double getOutput() {
        return this.controller.get();
    }

    public double getReference() {
        return this.lastReference;
    }

    public double getMeasurement() {
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
