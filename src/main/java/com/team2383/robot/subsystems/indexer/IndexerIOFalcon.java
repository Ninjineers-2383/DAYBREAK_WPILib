package com.team2383.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.team2383.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOFalcon implements IndexerIO {
    private final TalonFX m_indexer;

    private final DigitalInput m_beamBreak;

    private final StatusSignal<Double> m_supplyVoltage;
    private final StatusSignal<Double> m_supplyCurrent;

    public IndexerIOFalcon() {
        m_indexer = new TalonFX(IndexerConstants.kIndexerID, Constants.kCANivoreBus);
        m_indexer.setInverted(true);
        m_beamBreak = new DigitalInput(0);

        m_supplyVoltage = m_indexer.getSupplyVoltage();
        m_supplyCurrent = m_indexer.getSupplyCurrent();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(m_supplyVoltage, m_supplyCurrent).isOK();

        inputs.power = m_indexer.get();
        inputs.supplyVoltage = m_supplyVoltage.getValueAsDouble();
        inputs.supplyCurrent = m_supplyCurrent.getValueAsDouble();

        inputs.beamBreakTripped = !m_beamBreak.get();
    }

    @Override
    public void setPower(double power) {
        m_indexer.set(power);
    }
}
