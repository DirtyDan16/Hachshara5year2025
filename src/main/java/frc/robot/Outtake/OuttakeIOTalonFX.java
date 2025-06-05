package frc.robot.Outtake;

import com.ctre.phoenix6.hardware.TalonFX;

public class OuttakeIOTalonFX implements OuttakeIO{
    TalonFX controller;

    public OuttakeIOTalonFX() {
        controller = new TalonFX(0);
    }

    @Override
    public void setPercent(double percent) {
        controller.set(percent);
    }

    @Override
    public void stopOuttake() {
        controller.set(0);
    }

    @Override
    public boolean shouldStopOuttake(double angle) {
        return angle >= 50;
    }

    @Override
    public void updateInputs() {
        
    }

    
}
