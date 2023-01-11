package frc.robot;

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

    public static final CANAddress DRIVE_LEFT_FRONT = new CANAddress(1);
    public static final CANAddress DRIVE_LEFT_REAR = new CANAddress(2);
    public static final CANAddress DRIVE_RIGHT_FRONT = new CANAddress(3);
    public static final CANAddress DRIVE_RIGHT_REAR = new CANAddress(4);

    private Constants() {
        throw new UnsupportedOperationException();
    }
}
