package com.team2383.robot.subsystems.trap_arm;

import org.littletonrobotics.junction.AutoLog;

import com.team2383.lib.controller.TunableArmFeedforward;

import edu.wpi.first.math.controller.PIDController;

public interface TrapArmIO {
    @AutoLog
    public static class TrapArmIOInputs {
        public double velocityRadPerS = 0.0;
        public double pivotAngle = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
        public double desiredAngle = 0.0;
        public double desiredVelocity = 0.0;
        public double currentVelocity = 0.0;
        public double currentDesiredAngle = 0.0;
    }

    public default void updateInputs(TrapArmIOInputs inputs) {
    }

    public default void setAngle(double angle) {
    }

    public default void forceAngle(double angle) {
    }

    public default void setPIDController(PIDController controller) {
    }

    public default void setFeedforward(TunableArmFeedforward feedforward) {
    }

    public default void disable() {
    }
}