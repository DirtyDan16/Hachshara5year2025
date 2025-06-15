package frc.robot.Elevator;

import frc.lib.NinjasLib.controllers.*;
import frc.lib.NinjasLib.controllers.Controller.ControllerType;


public class ElevatorIOController implements ElevatorIO{
    Controller controller;
    public ElevatorIOController() {
        controller = Controller.createController(ControllerType.TalonFX, null);
    }

    @Override
    public void setHeight(double pos) {
        controller.setPosition(pos);
    }

    @Override
    public void stopElevator() {
        controller.setPercent(0);
    }

    @Override
    public boolean shouldStopElevator() {
        return controller.getPosition() > 2;
    }

    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    public void periodic() {
        controller.periodic();
    }

}
