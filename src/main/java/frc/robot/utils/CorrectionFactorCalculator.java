package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;

public class CorrectionFactorCalculator {
    /**
     *  Note: This value is in radians, and in our code, radians range from -PI to PI.
     *  So 0.8*PI = 0.9 rotations.
     */

    private static final double CALIBRATE_END = 0.8 * Math.PI;
    private static final int MIN_DATAPOINTS = 10;

    private static class DataPoint {
        public final double abs;
        public final double rel;

        public DataPoint(double abs, double rel) {
            this.abs = abs;
            this.rel = rel;
        }
    }

    private ArrayList<DataPoint> data = new ArrayList<>();

    private DoubleSupplier absEncoder;
    private DoubleSupplier relEncoder;

    double absZero;
    double relZero;

    public CorrectionFactorCalculator(DoubleSupplier absEncoder, DoubleSupplier relEncoder) {
        this.absEncoder = absEncoder;
        this.relEncoder = relEncoder;
    }

    public void start() {
        data.clear();
        absZero = absEncoder.getAsDouble();
        relZero = relEncoder.getAsDouble();
    }

    /**
     * Gets the current position of the absolute encoder, offset by the initial zero, in radians
     * constrained between [-pi, pi)
     * @returns The absolute position in radians
     */
    private double getAbsRad() {
        double rots = (absEncoder.getAsDouble() + 1 - absZero) % 1;
        return MoUtils.rotToRad(rots);
    }

    /**
     * Gets the current position of the relative encoder, offset by the initial zero, in radians.
     * @return The relative position in radians
     */
    private double getRelRad() {
        return relEncoder.getAsDouble() - relZero;
    }

    public boolean isFinished() {
        return getAbsRad() > CALIBRATE_END;
    }

    public void recordDataPoint() {
        DataPoint datum = new DataPoint(getAbsRad(), getRelRad());
        if(datum.abs > CALIBRATE_END) {
            return;
        }
        data.add(datum);
    }

    /**
     * Uses least-squares regression to calculate the factor the current relative encoder
     * scale should be adjusted by to minimize the error between the relative and absolute
     * encoders.
     * <p>
     * Let R be the relative encoder count, and A be the absolute position in radians (as given
     * by the absolute encoder). Also let B be the current encoder scale (in units of radians
     * per encoder count) and let F be the error factor such that R*B*F = A. It follows that
     * F = A / R*B.
     * Find the line of best fit where x = A and y = R*B. The slope of this line, m, estimates
     * R*B / A. Thus, F = 1/m, and so the correction factor is the reciprocal of the slope of
     * the line of best fit where the absolute position is the x-axis and the current estimated
     * position is the y-axis.
     *
     * @return The factor the current relative encoder scale should be adjusted by to minimize
     * the error between the relative and absolute encoders.
     */
    public double calculateCorrectionFactor() {
        double n = data.size();
        if(n < MIN_DATAPOINTS) {
            DriverStation.reportWarning("Cannot complete calibration: insufficient data", false);
            return 1;
        }

        double sumx = 0;
        double sumy = 0;
        double sumx2 = 0;
        double sumxy = 0;

        for(DataPoint point : data) {
            sumx += point.abs;
            sumy += point.rel;
            sumx2 += point.abs * point.abs;
            sumxy += point.abs * point.rel;
        }

        double lsrl_slope = ( (n * sumxy) - (sumx * sumy) ) / ( (n * sumx2) - (sumx * sumx) );

        return 1 / lsrl_slope;
    }
}
