package com.team2383.robot.commands.subsystem.drivetrain;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainHeadingCommand extends Command {
    private DrivetrainSubsystem drivetrain;
    private Rotation2d angle;

    public DrivetrainHeadingCommand(DrivetrainSubsystem drivetrain, Rotation2d angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        drivetrain.setHeading(angle);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.headingIsFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.endManualHeadingControl();
    }
}
