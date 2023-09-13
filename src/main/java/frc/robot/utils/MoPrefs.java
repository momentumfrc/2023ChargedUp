package frc.robot.utils;

import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringPublisher;

public final class MoPrefs {
    public static Pref<Double> maxDriveRpm = doublePref("Drive Max RPM", 6000.0);
    public static Pref<Double> maxDriveSpeed = doublePref("Drive Max Speed m/s", 6000.0);
    public static Pref<Double> maxTurnSpeed = doublePref("Turn Max Speed m/s", 6000.0);
    public static Pref<Double> driveDeadzone = doublePref("Drive Deadzone", 0.05);
    public static Pref<Double> driveCurve = doublePref("Drive Curve", 1);
    public static Pref<Double> driveSlowSpeed = doublePref("Drive Slow Speed", 0.5);
    public static Pref<Double> turnSlowSpeed = doublePref("Turn Slow Speed", 0.25);
    public static Pref<Double> driveRampTime = doublePref("Drive Ramp Time", 0.25);

    public static Pref<Double> flZero = doublePref("FL Drive Zero", 0);
    public static Pref<Double> flScale = doublePref("FL Drive Scale", 1);
    public static Pref<Double> frZero = doublePref("FR Drive Zero", 0);
    public static Pref<Double> frScale = doublePref("FR Drive Scale", 1);
    public static Pref<Double> rlZero = doublePref("RL Drive Zero", 0);
    public static Pref<Double> rlScale = doublePref("RL Drive Scale", 1);
    public static Pref<Double> rrZero = doublePref("RR Drive Zero", 0);
    public static Pref<Double> rrScale = doublePref("RR Drive Scale", 1);

    // Motor Scale: units of ticks per meter
    public static Pref<Double> flDriveMtrScale = doublePref("FL Drive Motor Scale", 1);
    public static Pref<Double> frDriveMtrScale = doublePref("FR Drive Motor Scale", 1);
    public static Pref<Double> rlDriveMtrScale = doublePref("RL Drive Motor Scale", 1);
    public static Pref<Double> rrDriveMtrScale = doublePref("RR Drive Motor Scale", 1);

    public static Pref<Double> shoulderMaxRpm = doublePref("Shoulder Maximum RPM", 200.0);
    public static Pref<Double> absShoulderZero = doublePref("Abs. Shoulder Zero", 0);
    public static Pref<Double> shoulderEncoderRatio = doublePref("Shoulder Ratio", 100 * 36 / 15);
    public static Pref<Double> shoulderMaxRevolutions = doublePref("Shouler Max (revs)", 0.25);

    public static Pref<Double> wristMaxRpm = doublePref("Wrist Maximum RPM", 200.0);
    public static Pref<Double> absWristZero = doublePref("Abs. Wrist Zero", 0);
    public static Pref<Double> wristEncoderRatio = doublePref("Wrist Ratio", 60 * 24 / 14);
    public static Pref<Double> wristMaxRevolutions = doublePref("Wrist Max (revs)", 0.8);

    public static Pref<Double> wristFallbackPower = doublePref("Wrist Fallback Max Power", 0.5);
    public static Pref<Double> armRampTime = doublePref("Arm Ramp Time", 0.25);

    public static Pref<Double> intakeSpeed = doublePref("Intake Speed", 1);

    public static Pref<Double> shoulderCurrentLimit = doublePref("Shoulder Current Limit", 10);
    public static Pref<Double> wristCurrentLimit = doublePref("Wrist Current Limit", 10);

    public static Pref<Double> autoScoreXDist = doublePref("Auto Score X Dist", 2.50);

    public static Pref<Double> chassisSizeX = doublePref("Chassis Size X", 1);
    public static Pref<Double> chassisSizeY = doublePref("Chassis Size Y", 1);

    public final class Pref<T> {
        public final String key;
        private Function<NetworkTableValue, T> getter;
        private BiFunction<NetworkTableEntry, T, Boolean> setter;

        private final NetworkTableEntry entry;

        private Consumer<T> subscriber = null;

        public Pref(String key, T defaultValue, Function<NetworkTableValue, T> getter, BiFunction<NetworkTableEntry, T, Boolean> setter) {
            this.key = key;
            this.getter = getter;
            this.setter = setter;

            this.entry = table.getEntry(key);
            this.entry.setDefaultValue(defaultValue);
            this.entry.setPersistent();
        }

        public T get() {
            return getter.apply(entry.getValue());
        }

        public void set(T value) {
            setter.apply(entry, value);
        }

        public void subscribe(Consumer<T> consumer) {
            subscribe(consumer, false);
        }

        public void subscribe(Consumer<T> consumer, boolean notifyImmediately) {
            if(subscriber != null) {
                subscriber = subscriber.andThen(consumer);
            } else {
                subscriber = consumer;
                entry.getInstance().addListener(
                    entry,
                    EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    (e) -> consumer.accept(getter.apply(e.valueData.value))
                );
            }

            if(notifyImmediately) {
                consumer.accept(this.get());
            }
        }
    }

    private static MoPrefs instance;
    private NetworkTable table;
    private StringPublisher typePublisher;

    private static MoPrefs getInstance() {
        if(instance == null) {
            instance = new MoPrefs();
        }
        return instance;
    }

    private MoPrefs() {
        table = NetworkTableInstance.getDefault().getTable("Preferences");
        typePublisher = table.getStringTopic(".type").publish();
        typePublisher.set("RobotPreferences");
    }

    private static Pref<Double> doublePref(String key, double defaultValue) {
        return getInstance().new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }
}
