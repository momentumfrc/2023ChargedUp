package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;

public class MoSparkMaxPID {
    private final Type type;
    private final CANSparkMax motorController;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private final int pidSlot;
    private double lastReference;

    private double kS = 0;
    private double positionWrappingRange = 0;

    public MoSparkMaxPID(Type type, CANSparkMax controller, int pidSlot) {
        this.type = type;
        this.motorController = controller;
        this.pidController = controller.getPIDController();
        this.encoder = controller.getEncoder();
        this.pidSlot = pidSlot;
    }

    public void setPositionPIDWrappingEnabled(double min, double max) {
        pidController.setPositionPIDWrappingEnabled(true);
        pidController.setPositionPIDWrappingMinInput(min);
        pidController.setPositionPIDWrappingMaxInput(max);
        this.positionWrappingRange = max - min;
    }

    public void setPositionPIDWrappingDisabled() {
        pidController.setPositionPIDWrappingEnabled(false);
        this.positionWrappingRange = 0;
    }

    public SparkMaxPIDController getPID() {
        return pidController;
    }

    public Type getType() {
        return type;
    }

    public int getPidSlot() {
        return pidSlot;
    }

    public void setP(double kP) {
        pidController.setP(kP, pidSlot);
    }

    public void setI(double kI) {
        pidController.setI(kI, pidSlot);
    }

    public void setD(double kD) {
        pidController.setD(kD, pidSlot);
    }

    public void setFF(double kFF) {
        pidController.setFF(kFF, pidSlot);
    }

    public void setIZone(double iZone) {
        pidController.setIZone(iZone, pidSlot);
    }

    public void setKS(double kS) {
        this.kS = kS;
    }

    public double getLastOutput() {
        return this.motorController.get();
    }

    public double getSetpoint() {
        return this.lastReference;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case POSITION_FF:
            case SMARTMOTION:
                return this.encoder.getPosition();
            case VELOCITY:
            case SMARTVELOCITY:
                return this.encoder.getVelocity();
        }

        return 0;
    }

    public void setReference(double target) {
        // In POSITION_FF, we apply an arbitrary feedfoward voltage to "linearize" the system
        // response. This kS can be determined using the wpilib SysId software.
        if(type == Type.POSITION_FF && kS != 0) {
            // We want to make sure the feedforward voltage isn't working against the PID
            // controller, so we need to anticipate which direction it will want to turn and apply a
            // positive or negative voltage as is appropriate.

            double curr = encoder.getPosition();
            double error = target - curr;

            // Calculating the error is a little bit more complicated when position wrapping
            // is enabled.
            if(positionWrappingRange != 0) {
                double errorBound = positionWrappingRange / 2;
                error = MathUtil.inputModulus(error, -errorBound, errorBound);
            }

            double ff = Math.signum(error) * kS;
            pidController.setReference(target, type.innerType, pidSlot, ff, SparkMaxPIDController.ArbFFUnits.kVoltage);
        } else {
            pidController.setReference(target, this.type.innerType, pidSlot, 0);
        }
        this.lastReference = target;
    }

    public enum Type {
        POSITION(CANSparkMax.ControlType.kPosition),
        POSITION_FF(CANSparkMax.ControlType.kPosition),
        SMARTMOTION(CANSparkMax.ControlType.kSmartMotion),
        VELOCITY(CANSparkMax.ControlType.kVelocity),
        SMARTVELOCITY(CANSparkMax.ControlType.kSmartVelocity);

        public final CANSparkMax.ControlType innerType;
        private Type(CANSparkMax.ControlType innerType) {
            this.innerType = innerType;
        }
    }
}
