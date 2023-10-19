package frc.robot.input;

import java.util.function.DoubleUnaryOperator;

public final class MoveVector {
    private double fwd;
    private double left;

    public MoveVector(double fwd, double left) {
        this.fwd = fwd;
        this.left = left;
    }

    void applyTransforms(DoubleUnaryOperator op) {
        double magnitude = Math.sqrt(fwd * fwd + left * left);
        if(magnitude > 0) {
            fwd /= magnitude;
            left /= magnitude;

            magnitude = op.applyAsDouble(magnitude);

            fwd *= magnitude;
            left *= magnitude;
        }
    }

    public double getFwd() {
        return fwd;
    }

    public double getLeft() {
        return left;
    }
}
