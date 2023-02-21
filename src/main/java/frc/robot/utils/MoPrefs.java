package frc.robot.utils;

import java.util.EnumSet;
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
    public static Pref<Double> driveDeadzone = doublePref("Drive Deadzone", 0.05);
    public static Pref<Double> driveCurve = doublePref("Drive Curve", 1);
    public static Pref<Double> driveSlowSpeed = doublePref("Drive Slow Speed", 0.5);
    public static Pref<Double> driveRampTime = doublePref("Drive Ramp Time", 0.25);
    public static Pref<Double> shoulderSetpointRpm = doublePref("Shoulder Speed RPM", 200.0); // TODO: Find actual value
    public static Pref<Double> wristSetpointRpm = doublePref("Wrist Speed RPM", 200.0); // TODO: Find actual value
    public static Pref<Double> armJointSetpointRatio = doublePref("Wrist/Shoulder Speed Ratio", 1); // TODO: Find actual value

    public final class Pref<T> {
        public final String key;
        private Function<NetworkTableValue, T> getter;

        private final NetworkTableEntry entry;

        private Consumer<T> subscriber = null;

        public Pref(String key, T defaultValue, Function<NetworkTableValue, T> getter) {
            this.key = key;
            this.getter = getter;

            this.entry = table.getEntry(key);
            this.entry.setDefaultValue(defaultValue);
            this.entry.setPersistent();
        }

        public T get() {
            return getter.apply(entry.getValue());
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
        return getInstance().new Pref<>(key, defaultValue, NetworkTableValue::getDouble);
    }
}
