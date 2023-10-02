package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MoTalonFxPID {
    private final Type type;
    private final TalonFX motorController;
    private double lastReference;

    public MoTalonFxPID(Type type, TalonFX controller) {
        this.type = type;
        this.motorController = controller;
    }

    public Type getType() {
        return type;
    }


    public void setP(double kP) {
        motorController.config_kP(0, kP);
    }

    public void setI(double kI) {
        motorController.config_kI(0, kI);
    }

    public void setD(double kD) {
        motorController.config_kD(0, kD);
    }

    public void setFF(double kFF) {
        motorController.config_kF(0, kFF);
    }

    public void setIZone(double iZone) {
        motorController.config_IntegralZone(0, iZone);
    }

    public double getLastOutput() {
        return this.motorController.getMotorOutputPercent();
    }

    public double getSetpoint() {
        return this.lastReference;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return this.motorController.getSelectedSensorPosition();
            case VELOCITY:
                return this.motorController.getSelectedSensorVelocity();
        }

        return 0;
    }

    public void setReference(double value) {
        this.motorController.set(this.type.innerType, value);
        this.lastReference = value;
    }

    public enum Type {
        POSITION(TalonFXControlMode.Position),
        SMARTMOTION(TalonFXControlMode.MotionMagic),
        VELOCITY(TalonFXControlMode.Velocity);

        public final TalonFXControlMode innerType;
        private Type(TalonFXControlMode innerType) {
            this.innerType = innerType;
        }
    }
}
