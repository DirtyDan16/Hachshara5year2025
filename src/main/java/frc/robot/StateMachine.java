package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.StateMachineBase;

import frc.robot.Elevator.*;
import frc.robot.Outtake.*;

public class StateMachine extends StateMachineBase<States>{

    @Override
    protected boolean canChangeRobotState(States currentState, States wantedState) {
        return switch(currentState) {
            case IDLE ->
                wantedState == States.PREPARE_OUTTAKE;
            case CLOSE -> 
                wantedState == States.IDLE;
            case PREPARE_OUTTAKE -> 
                wantedState == States.OUTTAKE;
            case OUTTAKE -> 
                wantedState == States.CLOSE;
        };
    }

    @Override
    protected void setCommandMap() {
        addCommand(States.IDLE, Commands.none());
        addCommand(States.PREPARE_OUTTAKE, 
            Commands.sequence(
                Commands.runOnce(() -> {Elevator.getInstance().io.setHeight(3);}),
                Commands.waitUntil(() -> {return Elevator.getInstance().io.shouldStopElevator();}),
                Commands.runOnce(() -> {Outtake.getInstance().io.setAngle(45);}),
                Commands.waitUntil(() -> {return Outtake.getInstance().io.shouldStopOuttake();})
            )
        );
        // addCommand(States.OUTTAKE,
        //     Commands.
        // );

        // addCommand(States.CLOSE, );
    }  
}
