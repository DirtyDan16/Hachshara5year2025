package frc.robot.Outtake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
    OuttakeIO io;
    double angle;

    public Outtake(OuttakeIO io) {
        angle = 0;
        this.io = io;
    }

    @Override
    public void periodic() {

    }

}
