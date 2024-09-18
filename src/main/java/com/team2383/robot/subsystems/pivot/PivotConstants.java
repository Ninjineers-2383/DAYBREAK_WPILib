package com.team2383.robot.subsystems.pivot;

public class PivotConstants {
    // CAN ID's
    public final static int kLeftMotorID = 2;
    public final static int kRightMotorID = 4;
    public final static int kEncoderID = 5;

    // Encoder Offset
    // 0.452881
    // 0.455811
    public final static double kEncoderOffset = -0.284912;

    // Feedback and Feedforward Gains
    // public final static ArmGains kGains = new ArmGains(20.0, 13.0, 2.0, 0.25,
    // 6.55, 0.0, 0.3, 0.0, 4.0);
    public final static ArmGains kGains = new ArmGains(15.0, 0.0, 0, 0.25, 10.5,
            0.0, 0.75, 0.0, 0.0);

    // Trapezoid Profile Constants (In rotations / s and rotations / s^2)
    public final static double kMaxVelo = 4.0;
    public final static double kMaxAccel = 1.25;

    // Gear Ratio
    public final static double kPivotMotorGearRatio = 1 / 85.33333;

    // Pivot Bounds
    public final static double kMaxAngleDegrees = 190;
    public final static double kMinAngleDegrees = -30;

    // Current Limits
    public final static double kCurrentLimit = 40;

    public record ArmGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG,
            double kSpring, double kBacklash) {
    }
}
