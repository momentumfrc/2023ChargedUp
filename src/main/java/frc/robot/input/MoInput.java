package frc.robot.input;

public interface MoInput {
    public double getForwardSpeedRequest();
    public double getLeftSpeedRequest();
    public double getTurnRequest();

    public boolean getShouldUseSlowSpeed();
    public boolean getShouldIntake();
    public boolean getShouldExhaust();

    public double getDirectShoulderRequest();
    public double getDirectWristRequest();

    public ArmPositionRequest getArmPositionRequest();
}
