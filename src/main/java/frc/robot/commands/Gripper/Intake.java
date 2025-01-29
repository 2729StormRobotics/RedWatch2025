package frc.robot.commands.Gripper;

import frc.robot.subsystems.Gripper.GripperIO;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    private final GripperIO m_gripper;

    public Intake(GripperIO gripper) {
        // Makes the gripper object for the command
        m_gripper = gripper;
    }

    @Override
    public boolean isFinished() {
        // returns what the isCoralPresent method returns
        return !m_gripper.isCoralPresent();
    }

    @Override
    public void execute(){
        // sets the gripper object to setIn 
        m_gripper.setIn();
    }

    @Override
    public void end(boolean interrupted) {
        // sets the gripper object to stop
        m_gripper.stop();
    }
}