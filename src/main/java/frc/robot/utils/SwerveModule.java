package frc.robot.utils;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.momentum4999.utils.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.utils.MoPrefs.Pref;

public class SwerveModule {
    private static final double ABSOLUTE_ENCODER_SCALE = 1/3.3;

    private final String key;
    public final CANSparkMax turnMotor;
    public final WPI_TalonFX driveMotor;

    // Note: the absolute encoder returns rotations, in the range [0, 1)
    public final SparkMaxAnalogSensor absoluteEncoder;

    private final MoSparkMaxPID turnPID;
    private final MoTalonFxPID drivePID;

    private PIDTuner turnTuner;
    private PIDTuner driveTuner;

    // Note: the relative encoder is scaled to return radians
    private final RelativeEncoder relativeEncoder;

    private final Pref<Double> encoderZero;
    private final Pref<Double> encoderScale;
    private final Pref<Double> driveMtrScale;

    public SwerveModule(String key, CANSparkMax turnMotor, WPI_TalonFX driveMotor, Pref<Double> encoderZero, Pref<Double> encoderScale, Pref<Double> driveMtrScale, boolean showPidTuners) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.encoderZero = encoderZero;
        this.encoderScale = encoderScale;
        this.driveMtrScale = driveMtrScale;

        this.driveMotor.setNeutralMode(NeutralMode.Brake);

        this.absoluteEncoder = turnMotor.getAnalog(Mode.kAbsolute);
        this.absoluteEncoder.setPositionConversionFactor(ABSOLUTE_ENCODER_SCALE);

        this.turnPID = new MoSparkMaxPID(MoSparkMaxPID.Type.POSITION, turnMotor, 0);
        this.drivePID = new MoTalonFxPID(MoTalonFxPID.Type.VELOCITY, driveMotor);

        var turnSparkMaxPID = turnPID.getPID();
        turnSparkMaxPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);

        turnTuner = TunerUtils.forMoSparkMax(turnPID, key + "_turn", !showPidTuners);
        driveTuner = TunerUtils.forMoTalonFx(drivePID, key + "_drive", !showPidTuners);

        relativeEncoder = turnMotor.getEncoder();

        encoderZero.subscribe(zero -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), zero, encoderScale.get()), false);
        encoderScale.subscribe(scale -> this.setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), scale), false);
        setupRelativeEncoder();

        var layout = Shuffleboard.getTab("match").getLayout(key, BuiltInLayouts.kList)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "LEFT"));
        layout.addDouble("Relative", () -> (MoUtils.radToRot(relativeEncoder.getPosition())));
        layout.addDouble("Absolute", absoluteEncoder::getPosition);
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(absoluteEncoder.getPosition(), encoderZero.get(), encoderScale.get());
    }

    public void setRelativePosition() {
        setRelativePosition(absoluteEncoder.getPosition(), encoderZero.get());
    }

    private void setupRelativeEncoder(double absPos, double absZero, double scale) {
        relativeEncoder.setPositionConversionFactor(scale);
        relativeEncoder.setVelocityConversionFactor(scale);

        setRelativePosition(absPos, absZero);
    }

    private void setRelativePosition(double absPos, double absZero) {
        double rots = absPos;
        rots = (rots + 1 - absZero) % 1;
        relativeEncoder.setPosition(MoUtils.rotToRad(rots));
    }

    public void drive(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, Rotation2d.fromRadians(relativeEncoder.getPosition()));
        turnPID.setReference(MathUtil.angleModulus(optimized.angle.getRadians()));
        drivePID.setReference(optimized.speedMetersPerSecond * driveMtrScale.get());
    }

    public void directDrive(double turnSpeed, double driveSpeed) {
        turnMotor.set(turnSpeed);
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
    }

    @Override
    public String toString() {
        return String.format("SwerveModule(key=\"%s\")", this.key);
    }
}
