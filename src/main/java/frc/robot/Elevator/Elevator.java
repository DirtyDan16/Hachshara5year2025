package frc.robot.Elevator;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    public ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs;

    static Elevator instance;


    public Elevator(ElevatorIO io) {
        inputs = new ElevatorIOInputsAutoLogged();
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        io.periodic();
    }

    public static void createElevator(ElevatorIO io) {
        if (instance == null) instance = new Elevator(io);
    }

    public static Elevator getInstance() {
        return instance;
    }
}
