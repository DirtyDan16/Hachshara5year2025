package frc.robot.Elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.NinjasLib.controllers.Controller;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs extends Controller.ControllerIOInputs {
    }

    public void setHeight(double pos);
    public void stopElevator();
    public boolean shouldStopElevator();
    
    public void updateInputs(ElevatorIOInputsAutoLogged inputs);

    public void periodic();

}
