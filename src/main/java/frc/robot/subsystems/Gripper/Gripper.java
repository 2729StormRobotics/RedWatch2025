package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GripperInCommand;


public class Gripper extends SubsystemBase {
    private GripperIO io;

    public Gripper(GripperIO io) {
        this.io = io;
    }

    public boolean isCoralPresent() {
        return io.isCoralPresent();
    }


    public void stop() {
        io.stop();
    }

    public void setIn() {
        new GripperInCommand(this).schedule();
    }

    public void setOut() { 
        new GripperInCommand(this).schedule();
    }
}
