package frc.robot.Outtake;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.NinjasLib.controllers.Controller;

public interface OuttakeIO {
    @AutoLog
    class OuttakeIOInputs extends Controller.ControllerIOInputs {
    }

    public void setPercent(double percent);
    public void stopOuttake();
    public boolean shouldStopOuttake(double angle);
    
    public void updateInputs();

}
