package frc.robot.commands.Gripper;

import frc.robot.subsystems.Gripper.GripperIO;
import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends Command {
    // Same command as Intake except that excute using setOut, reversing the direction
    private final GripperIO m_gripper;

    public Outtake(GripperIO gripper) {
        m_gripper = gripper;
    }

    @Override
    public boolean isFinished() {
        return !m_gripper.isCoralPresent();
    }

    @Override
    public void execute(){
        m_gripper.setOut();
    }

    @Override
    public void end(boolean interrupted) {
        m_gripper.stop();
    }
}