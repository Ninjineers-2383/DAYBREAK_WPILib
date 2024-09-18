package com.team2383.robot.subsystems.resting_hooks;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class RestingHookIOSparkMax implements RestingHookIO {
    private final CANSparkMax motor;

    public RestingHookIOSparkMax() {
        motor = new CANSparkMax(RestingHookConstants.kLeftHookID, MotorType.kBrushless);

        motor.setInverted(true);
    }

    @Override
    public void updateInputs(RestingHookIOInputs inputs) {
        inputs.leftHookConnected = true;
        inputs.rightHookConnected = false;

        inputs.voltage = motor.getAppliedOutput();
        inputs.current = motor.getOutputCurrent();
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void setPowerSingle(double power) {
        motor.set(power);
    }
}
