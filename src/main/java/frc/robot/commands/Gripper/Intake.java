package frc.robot.commands.Gripper;

import frc.robot.subsystems.Gripper.GripperIO;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    private final GripperIO m_gripper;

    public Intake(GripperIO gripper) {
        m_gripper = gripper;
    }

    @Override
    public boolean isFinished() {
        return !m_gripper.isCoralPresent();
    }

    @Override
    public void execute(){
        m_gripper.setIn();
    }

    @Override
    public void end(boolean interrupted) {
        m_gripper.stop();
    }
}