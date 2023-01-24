package frc.robot.utils;

import com.momentum4999.utils.PIDTuner.PIDAdapter;
import com.playingwithfusion.CANVenom;

public class VenomTunerAdapter implements PIDAdapter {
    private final CANVenom venom;

    public VenomTunerAdapter(CANVenom venom) {
        this.venom = venom;
    }

    @Override
    public void setP(double kP) {
        venom.setKP(kP);
    }

    @Override
    public void setI(double kI) {
        venom.setKI(kI);
    }

    @Override
    public void setD(double kD) {
        venom.setKD(kD);
    }

    @Override
    public void setFF(double kFF) {
        venom.setKF(kFF);
    }

    @Override
    public void setIZone(double kIZone) {
        if(kIZone != 0) {
            System.err.println("Warning: attempt to set unsupported kIZone on a CANVenom");
        }
    }

    @Override
    public double getSetpoint() {
        return venom.getPIDTarget();
    }

    @Override
    public double getCurrentValue() {
        return venom.getSpeed();
    }

}
