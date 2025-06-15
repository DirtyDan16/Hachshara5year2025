package frc.robot.Outtake;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.NinjasLib.controllers.Controller;

public interface OuttakeIO {
    @AutoLog
    class OuttakeIOInputs extends Controller.ControllerIOInputs {
        // double Angle;
    }

    public void setAngle(double angle);
    public void stopOuttake();
    public boolean shouldStopOuttake();
    
    public void updateInputs(OuttakeIOInputsAutoLogged inputs);
    
    public void periodic();
}    

