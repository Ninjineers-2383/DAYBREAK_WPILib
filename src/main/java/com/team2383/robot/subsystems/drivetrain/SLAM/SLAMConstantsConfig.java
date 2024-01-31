package com.team2383.robot.subsystems.drivetrain.SLAM;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class SLAMConstantsConfig {
    // Description of the camera's relative position
    private static final Rotation3d CAM_ROTATION_FL = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-20));
    private static final Rotation3d CAM_ROTATION_FR = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-110));
    private static final Rotation3d CAM_ROTATION_BL = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-290));
    private static final Rotation3d CAM_ROTATION_BR = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-200));

    public static final double POSE_VARIANCE_SCALE = 0.01;
    public static final double POSE_VARIANCE_STATIC = 1E-3;

    private static final Translation3d CAM_FRONT_LEFT = new Translation3d(
            0.257 + Units.inchesToMeters(1),
            0.244 + Units.inchesToMeters(1),
            0.163);

    private static final Translation3d CAM_FRONT_RIGHT = new Translation3d(
            0.243 + Units.inchesToMeters(1),
            -0.255 - Units.inchesToMeters(1),
            0.163);

    private static final Translation3d CAM_BACK_LEFT = new Translation3d(
            -0.244 - Units.inchesToMeters(
                    1),
            0.263 + Units.inchesToMeters(
                    1),
            0.163);

    private static final Translation3d CAM_BACK_RIGHT = new Translation3d(
            -0.263 - Units.inchesToMeters(
                    1),
            -0.244 - Units.inchesToMeters(
                    1),
            0.163);

    public static final Transform3d[] camTransforms = new Transform3d[] {
            new Transform3d(CAM_FRONT_LEFT, CAM_ROTATION_FL),
            new Transform3d(CAM_FRONT_RIGHT, CAM_ROTATION_FR),
            new Transform3d(CAM_BACK_LEFT, CAM_ROTATION_BL),
            new Transform3d(CAM_BACK_RIGHT, CAM_ROTATION_BR)
    };
}
