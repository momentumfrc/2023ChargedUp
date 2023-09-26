package frc.robot.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;

/**
 * We only want to use the absolute encoder to re-zero the relative encoder if four conditions are
 * true:
 * <ol>
 * <li>The current encoder reading is <i>valid</i>, as defined by the isAbsEncoderValid supplier.</li>
 * <li>We have already seen NUM_MEAN_SAMPLES <i>consecutive</i> valid encoder readings.</li>
 * <li>It has been at least RESET_ENCODER_INTERVAL seconds since the last time the encoders were zeroed.</li>
 * <li>Encoder zeroing has not been disabled.</li>
 * </ol>
 *
 * Since the absolute encoder uses an absolute voltage to return its value, errors can be introduced
 * by electromagnetic interference produced by the motor. To account for this, isAbsEncoderValid
 * indicates that there is no commanded motion on nearby motors. Also, since we take the mean of the
 * last NUM_MEAN_SAMPLES values, the encoder reading will be inaccurate if the motor is actually
 * moving.
 *
 */
public class FusedEncoders {
    private static final int NUM_MEAN_SAMPLES = 5;
    private static final double RESET_ENCODER_INTERVAL = 0.5;

    private final Supplier<Double> absPosGetter;
    private final Consumer<Double> absPosConsumer;

    private final Supplier<Boolean> isAbsEncoderValid;

    private final MedianFilter filter = new MedianFilter(NUM_MEAN_SAMPLES);
    private int nSamples = 0;

    private final Timer timer = new Timer();
    private boolean timerStarted = false;

    private boolean shouldAutoZero = false;

    public FusedEncoders(Supplier<Double> absPosGetter, Consumer<Double> absPosConsumer, Supplier<Boolean> isAbsEncoderValid) {
        this.absPosGetter = absPosGetter;
        this.absPosConsumer = absPosConsumer;

        this.isAbsEncoderValid = isAbsEncoderValid;
    }

    void setShouldAutoZero(boolean shouldAutoZero) {
        this.shouldAutoZero = shouldAutoZero;
    }

    void periodic() {
        if(isAbsEncoderValid.get()) {
            double value = filter.calculate(absPosGetter.get());

            if(nSamples < NUM_MEAN_SAMPLES) {
                nSamples += 1;
            } else if (shouldAutoZero && (!timerStarted || timer.hasElapsed(RESET_ENCODER_INTERVAL))) {
                timerStarted = true;
                timer.restart();
                absPosConsumer.accept(value);
            }
        } else if(nSamples > 0) {
            filter.reset();
            nSamples = 0;
        }
    }

}
