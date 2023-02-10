package frc.robot;

import java.io.File;

import com.momentum4999.utils.PIDTuner.PIDTunerSettings;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    /**
     * A wrapper class holding a single integer value representing a CAN Address. The point of this
     * class is to indicate that the wrapped value is a CAN Address in a more robust way than
     * just adding "CAN_ADDR" to the constant's name.
    */
    public static class CANAddress {
        public final int address;
        public CANAddress(int address) {
            this.address = address;
        }
    }

    /**
     * A wrapper class holding a single integer value representing a HID Port. The point of this
     * class is to indicate that the wrapped value is a HID Port in a more robust way than
     * just adding "PORT" to the constant's name.
    */
    public static class HIDPort {
        public final int port;
        public HIDPort(int port) {
            this.port = port;
        }
    }

    public static final CANAddress DRIVE_LEFT_FRONT = new CANAddress(3);
    public static final CANAddress DRIVE_LEFT_REAR = new CANAddress(4);
    public static final CANAddress DRIVE_RIGHT_FRONT = new CANAddress(1);
    public static final CANAddress DRIVE_RIGHT_REAR = new CANAddress(5);

    public static final HIDPort F310 = new HIDPort(0);

    public static final PIDTunerSettings TUNER_SETTINGS = new PIDTunerSettings();

    static {
        if(RobotBase.isReal()) {
            TUNER_SETTINGS.saveValuesLocation = new File("/home/lvuser/pid_constants.ini");
        }
    }

    private Constants() {
        throw new UnsupportedOperationException();
    }
}
