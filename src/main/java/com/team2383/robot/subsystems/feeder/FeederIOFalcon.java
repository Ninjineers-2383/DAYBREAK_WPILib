package com.team2383.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIOFalcon implements FeederIO {
    private final TalonFX feeder;

    private final VoltageOut voltageOut = new VoltageOut(0);

    private final StatusSignal<Double> current;

    private final DigitalInput m_beamBreak;

    private double voltage = 0;

    public FeederIOFalcon(int id, String canbus) {
        feeder = new TalonFX(id, canbus);

        m_beamBreak = new DigitalInput(1);

        current = feeder.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, current);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(current).isOK();
        inputs.current = current.getValueAsDouble();
        inputs.power = voltage;

        inputs.beamBreakTripped = !m_beamBreak.get();
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        feeder.setControl(voltageOut.withOutput(-voltage));

    }
}
