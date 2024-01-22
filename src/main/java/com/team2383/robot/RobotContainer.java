// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.team2383.robot.Constants.Mode;
import com.team2383.robot.commands.IndexerCommand;
import com.team2383.robot.commands.JoystickDriveCommand;
import com.team2383.robot.commands.OrchestraCommand;
import com.team2383.robot.commands.ShooterRPMCommand;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.drivetrain.GyroIO;
import com.team2383.robot.subsystems.drivetrain.GyroIOPigeon;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOFalcon500;
import com.team2383.robot.subsystems.drivetrain.SwerveModuleIOSim;
import com.team2383.robot.subsystems.indexer.IndexerIONEO;
import com.team2383.robot.subsystems.indexer.IndexerIOSim;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterIOFalcon500Neo;
import com.team2383.robot.subsystems.shooter.ShooterIOSim;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final GenericHID m_driverController = new GenericHID(0);

    private DrivetrainSubsystem m_drivetrainSubsystem;

    private IndexerSubsystem m_indexerSubsystem;

    private ShooterSubsystem m_shooterSubsystem;

    LoggedDashboardChooser<Boolean> enableLW = new LoggedDashboardChooser<Boolean>("Enable LW");

    LoggedDashboardNumber shooterTopBottomRPM = new LoggedDashboardNumber("Top Bottom RPM", 0);
    LoggedDashboardNumber shooterSideRPM = new LoggedDashboardNumber("Side RPM", 0);

    LoggedDashboardChooser<Command> testDashboardChooser = new LoggedDashboardChooser<Command>("Test Command");

    private boolean lwEnabled = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case ROBOT_PROTO:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIOPigeon(5, "drive"),
                            new SwerveModuleIOFalcon500(DriveConstants.frontLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.frontRightConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearLeftConstants,
                                    Constants.kCANivoreBus),
                            new SwerveModuleIOFalcon500(DriveConstants.rearRightConstants,
                                    Constants.kCANivoreBus));

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIONEO());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOFalcon500Neo());
                    break;
                case ROBOT_SIM:
                    m_drivetrainSubsystem = new DrivetrainSubsystem(
                            new GyroIO() {},
                            new SwerveModuleIOSim(), new SwerveModuleIOSim(),
                            new SwerveModuleIOSim(), new SwerveModuleIOSim());

                    m_indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());

                    m_shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                    break;
                default:
                    break;
            }
        }

        configureDefaultCommands();
        registerAutoCommands();
        registerTestCommands();
        // Configure the button bindings
        configureButtonBindings();

        enableLW.addDefaultOption("No", false);
        enableLW.addOption("Yes", true);

    }

    public void periodic() {
        // autoChooser.periodic();

        if (enableLW.get() && !lwEnabled) {
            LiveWindow.enableAllTelemetry();
            lwEnabled = true;
        } else if (!enableLW.get() && lwEnabled) {
            lwEnabled = false;
            LiveWindow.disableAllTelemetry();
        }
    }

    private void configureButtonBindings() {

    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(
                new JoystickDriveCommand(m_drivetrainSubsystem,
                        () -> new Translation2d(
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX), .1),
                                MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY), .1)),
                        () -> Rotation2d
                                .fromDegrees(100 * MathUtil
                                        .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1)),
                        () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric)),
                        () -> -1));

        m_shooterSubsystem.setDefaultCommand(
                new ShooterRPMCommand(m_shooterSubsystem, shooterTopBottomRPM::get, shooterSideRPM::get));

        m_indexerSubsystem
                .setDefaultCommand(new IndexerCommand(m_indexerSubsystem, () -> m_driverController.getRawAxis(3)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return (Command) null;
    }

    public Command getTestCommand() {
        return testDashboardChooser.get();
    }

    private void registerAutoCommands() {
    }

    private void registerTestCommands() {
        testDashboardChooser.addDefaultOption("None", (Command) null);

        testDashboardChooser.addOption("Shooter Quasi Static Forward",
                m_shooterSubsystem.getQuasiStatic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Quasi Static Reverse",
                m_shooterSubsystem.getQuasiStatic(Direction.kReverse));

        testDashboardChooser.addOption("Shooter Dynamic Forward",
                m_shooterSubsystem.getDynamic(Direction.kForward));

        testDashboardChooser.addOption("Shooter Dynamic Reverse",
                m_shooterSubsystem.getDynamic(Direction.kReverse));

        testDashboardChooser.addOption("Sea Shanty 2", new OrchestraCommand("SeaShanry2.chrp"));
    }
}
