package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootIfAble extends Command {
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Shooting", true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooting", false);
    }
}
