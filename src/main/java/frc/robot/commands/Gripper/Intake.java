package frc.robot.commands.Gripper;

import frc.robot.subsystems.Gripper.Gripper;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    private final Gripper m_gripper;

    public Intake(Gripper gripper) {
        m_gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        m_gripper.setIn();
    }

    @Override
    public boolean isFinished() {
        return m_gripper.isCoralPresent();
    }

    @Override
    public void end(boolean interrupted) {
        m_gripper.stop();
    }
}