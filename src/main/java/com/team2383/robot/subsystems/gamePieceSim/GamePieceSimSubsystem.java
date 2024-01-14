package com.team2383.robot.subsystems.gamePieceSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class GamePieceSimSubsystem extends SubsystemBase {
    private final Supplier<Pose3d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private BooleanSupplier shootingSupplier;
    private BooleanSupplier intakeSupplier;

    private Pose3d[] notes = GamePieceLocations.notes;
    private boolean[] notesHit = new boolean[notes.length];

    public GamePieceSimSubsystem(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier,
            BooleanSupplier shootingSupplier, BooleanSupplier intakeSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.shootingSupplier = shootingSupplier;
        this.intakeSupplier = intakeSupplier;
    }

    @Override
    public void periodic() {
        Pose3d pose = poseSupplier.get();
        boolean intake = intakeSupplier.getAsBoolean();
        boolean shooting = shootingSupplier.getAsBoolean();
        ChassisSpeeds speeds = speedsSupplier.get();

        for (int i = 0; i < notes.length; i++) {
            if (pose.getTranslation().getDistance(notes[i].getTranslation()) < 0.5
                    && notesHit[i] == false) {
                int collisionSide = getCollisionSide(pose, notes[i]);

                if (intake) {
                    if (collisionSide == 2 || collisionSide == 3) {
                        notes[i] = new Pose3d(new Translation3d(pose.getX(), pose.getY(), 1),
                                new Rotation3d(0, Math.toRadians(-90), 0));

                        notesHit[i] = true;
                    } else {
                        notesHit[i] = false;
                    }
                }

                movePiece(i, collisionSide, pose);
            }

            if (notesHit[i] == true) {
                if (shooting) {
                    notes[i] = new Pose3d(new Translation3d(pose.getX(), pose.getY(), 1),
                            new Rotation3d(0, Math.toRadians(-90), 0));
                } else {
                    notes[i] = new Pose3d(new Translation3d(pose.getX(), pose.getY(), 0.2),
                            new Rotation3d(0, Math.toRadians(-90), 0));
                }
            }

            Logger.recordOutput("Notes", notes);
        }
    }

    public void movePiece(int noteIndex, int collisionSide, Pose3d robotPose) {
        ChassisSpeeds speeds = speedsSupplier.get();

        Rotation2d speedsAngle = new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond))
                .plus(new Rotation2d(Math.PI)).plus(robotPose.getRotation().toRotation2d());

        double speedsAngleDegrees = speedsAngle.getDegrees();

        boolean move = false;
        switch (collisionSide) {
            case 1:
                if (speedsAngleDegrees > -80 && speedsAngleDegrees < 80) {
                    move = true;
                }
                break;
            case 2:
                if (speedsAngleDegrees > 10 && speedsAngleDegrees < 170) {
                    move = true;
                }
                break;
            case 3:
                if (speedsAngleDegrees < -10 && speedsAngleDegrees > -170) {
                    move = true;
                }
                break;
            case 4:
                if (speedsAngleDegrees > 80 || speedsAngleDegrees < -80) {
                    move = true;
                }
                break;
            default:
                break;

        }
        if (move) {
            notes[noteIndex] = notes[noteIndex]
                    .exp(new Twist3d(0, speeds.vyMetersPerSecond * 0.02, -speeds.vxMetersPerSecond * 0.02, 0, 0, 0));
        }

        Logger.recordOutput("Speeds Angle", speedsAngleDegrees);

    }

    public Rotation2d getCollisionAngle(Pose3d robotPose, Pose3d notePose) {
        return new Rotation2d(Math.atan2(robotPose.getY() - notePose.getY(), robotPose.getX() - notePose.getX()))
                .plus(robotPose.getRotation().toRotation2d());
    }

    public int getCollisionSide(Pose3d robotPose, Pose3d notePose) {
        Rotation2d collisionAngle = getCollisionAngle(robotPose, notePose);
        double collisionAngleDegrees = collisionAngle.getDegrees();
        int side = 0;

        if (collisionAngleDegrees > -45 && collisionAngleDegrees < 45) {
            side = 1;
        } else if (collisionAngleDegrees > 45 && collisionAngleDegrees < 135) {
            side = 2;
        } else if (collisionAngleDegrees < -45 && collisionAngleDegrees > -135) {
            side = 3;
        } else if (collisionAngleDegrees > 135 || collisionAngleDegrees < -135) {
            side = 4;
        }

        Logger.recordOutput("Collision Side", side);
        Logger.recordOutput("Collision Angle", collisionAngleDegrees);
        return side;
    }
}