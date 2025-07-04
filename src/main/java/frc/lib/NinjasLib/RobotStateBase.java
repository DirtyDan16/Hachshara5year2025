package frc.lib.NinjasLib;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public abstract class RobotStateBase<StateEnum> {
    protected static RobotStateBase instance;
    protected StateEnum robotState;

    public static RobotStateBase getInstance() {
        if (instance == null)
            throw new RuntimeException("RobotStateIO not initialized. Initialize RobotStateIO by setInstance() first.");
        return instance;
    }

    public static void setInstance(RobotStateBase instance) {
        RobotStateBase.instance = instance;
    }

    /**
     * @return State of the robot
     */
    public StateEnum getRobotState() {
        return robotState;
    }

    /**
     * Sets the state of the robot to the given state
     *
     * @param state The state to set the robot state to
     */
    public void setRobotState(StateEnum state) {
        System.out.println("[Robot State Change] " + robotState.toString() + " -> " + state.toString());
        Logger.recordOutput("Robot State", state.toString());
        robotState = state;
    }

    /**
     * @return Whether the robot is in the blue alliance or the red alliance
     */
    public static DriverStation.Alliance getAlliance() {
        return Robot.isSimulation()
            ? (DriverStationSim.getAllianceStationId().ordinal() > 3
            ? DriverStation.Alliance.Blue
            : DriverStation.Alliance.Red)
            : DriverStation.getAlliance().get();
    }

    /**
     * @return Which driver station the robot is in
     */
    public AllianceStationID getAllianceStation() {
        return Robot.isSimulation() ? DriverStationSim.getAllianceStationId() : DriverStation.getRawAllianceStation();
    }
}
