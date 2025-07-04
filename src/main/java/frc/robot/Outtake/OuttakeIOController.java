package frc.robot.Outtake;

import frc.lib.NinjasLib.controllers.*;
import frc.lib.NinjasLib.controllers.Controller.ControllerType;


public class OuttakeIOController implements OuttakeIO{
    Controller controller;
    public OuttakeIOController() {
        controller = Controller.createController(ControllerType.TalonFX, null);
    }

    @Override
    public void setAngle(double percent) {
        controller.setPosition(percent);
    }

    @Override
    public void stopOuttake() {
        controller.setPercent(0);
    }

    @Override
    public boolean shouldStopOuttake() {
        return true;
    }

    @Override
    public void updateInputs(OuttakeIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
        // inputs.Angle = getAngle();
    }
    public void periodic() {
        controller.periodic();
    }
}
