package frc.robot.Outtake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Outtake extends SubsystemBase {
    public OuttakeIO io;
    OuttakeIOInputsAutoLogged inputs;

    static Outtake instance;


    public Outtake(OuttakeIO io) {
        inputs = new OuttakeIOInputsAutoLogged();
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Outtake", inputs);
        io.periodic();
    }

    public static void createOuttake(OuttakeIO io) {
        if (instance == null) instance = new Outtake(io);
    }
    
    public static Outtake getInstance() {
        return instance;
    }
}