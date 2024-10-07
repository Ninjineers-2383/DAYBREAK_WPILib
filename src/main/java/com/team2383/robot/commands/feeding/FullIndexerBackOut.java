package com.team2383.robot.commands.feeding;

import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FullIndexerBackOut extends ParallelCommandGroup {
    public FullIndexerBackOut(ShooterSubsystem shooter, IndexerSubsystem indexer) {

        addCommands(new ShooterRPMCommand(shooter, () -> 400, () -> 0, () -> 0, true), new IndexerBackOut(indexer));
    }
}
