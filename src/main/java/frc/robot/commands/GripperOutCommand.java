package frc.robot.commands;

import frc.robot.subsystems.Gripper.Gripper;
import edu.wpi.first.wpilibj2.command.Command;

public class GripperOutCommand extends Command {
    private final Gripper m_gripper;

    public GripperOutCommand(Gripper gripper) {
        m_gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        m_gripper.setOut();
    }

    @Override
    public boolean isFinished() {
        return !m_gripper.isCoralPresent();
    }

    @Override
    public void end(boolean interrupted) {
        m_gripper.stop();
    }
}