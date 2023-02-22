package frc.robot.input;

public interface MoInput {
    public double getForwardSpeedRequest();
    public double getLeftSpeedRequest();
    public double getTurnRequest();

    public boolean getShouldUseSlowSpeed();

    public double getDirectShoulderRequest();
    public double getDirectWristRequest();
}
