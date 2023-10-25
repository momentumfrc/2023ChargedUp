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
    private double smartMotionMotionAllowedClosedLoopError = 0;

    public MoSparkMaxPID(Type type, CANSparkMax controller, int pidSlot) {
        this.type = type;
        this.motorController = controller;
        this.pidController = controller.getPIDController();
        this.encoder = controller.getEncoder();
        this.pidSlot = pidSlot;

        if(pidController.getPositionPIDWrappingEnabled()) {
            positionWrappingRange = pidController.getPositionPIDWrappingMaxInput() - pidController.getPositionPIDWrappingMinInput();
        } else {
            positionWrappingRange = 0;
        }

        smartMotionMotionAllowedClosedLoopError = pidController.getSmartMotionAllowedClosedLoopError(pidSlot);
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

    public void setSmartMotionAllowedClosedLoopError(double error) {
        this.smartMotionMotionAllowedClosedLoopError = error;
        pidController.setSmartMotionAllowedClosedLoopError(error, pidSlot);
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
            case SMARTMOTION:
            case SMARTMOTION_KS:
                return this.encoder.getPosition();
            case VELOCITY:
            case VELOCITY_KS:
            case SMARTVELOCITY:
                return this.encoder.getVelocity();
        }

        return 0;
    }

    public void setReference(double target) {
        // In SMARTMOTION_KS, we apply an arbitrary feedfoward voltage to "linearize" the system
        // response. This kS can be determined using the wpilib SysId software.
        if(type == Type.SMARTMOTION_KS && kS != 0) {
            // We want to make sure the feedforward voltage isn't working against the
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

            // If we're within the allowed error, we shouldn't move, so don't apply the feedforward
            if(Math.abs(error) < smartMotionMotionAllowedClosedLoopError) {
                error = 0;
            }

            double ff = Math.signum(error) * kS;
            pidController.setReference(target, type.innerType, pidSlot, ff, SparkMaxPIDController.ArbFFUnits.kVoltage);
        } else if(type == Type.VELOCITY_KS && kS != 0) {
            double error = target - encoder.getVelocity();
            double ff = Math.signum(error) * kS;
            pidController.setReference(target, type.innerType, pidSlot, ff, SparkMaxPIDController.ArbFFUnits.kVoltage);
        } else {
            pidController.setReference(target, this.type.innerType, pidSlot, 0);
        }
        this.lastReference = target;
    }

    public enum Type {
        POSITION(CANSparkMax.ControlType.kPosition),
        SMARTMOTION(CANSparkMax.ControlType.kSmartMotion),
        SMARTMOTION_KS(CANSparkMax.ControlType.kSmartMotion),
        VELOCITY(CANSparkMax.ControlType.kVelocity),
        VELOCITY_KS(CANSparkMax.ControlType.kVelocity),
        SMARTVELOCITY(CANSparkMax.ControlType.kSmartVelocity);

        public final CANSparkMax.ControlType innerType;
        private Type(CANSparkMax.ControlType innerType) {
            this.innerType = innerType;
        }
    }
}
