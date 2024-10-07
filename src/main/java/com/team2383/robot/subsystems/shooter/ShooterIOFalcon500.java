package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.team2383.robot.Constants;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import java.util.List;

public class ShooterIOFalcon500 implements ShooterIO {
    private final TalonFX topMotor = new TalonFX(ShooterConstants.kTopMotorID, Constants.kCANivoreBus);
    private final TalonFX bottomMotor = new TalonFX(ShooterConstants.kBottomMotorID, Constants.kCANivoreBus);

    private final VelocityVoltage voltageOut = new VelocityVoltage(0);

    private double topSetpoint = 0.0;
    private double bottomSetpoint = 0.0;
    private double sideSetpoint = 0.0;

    private final List<StatusSignal<Double>> topBottomSupplyCurrent;
    private final List<StatusSignal<Double>> topBottomSupplyVoltage;
    private final List<StatusSignal<Double>> topBottomPosition;
    private final List<StatusSignal<Double>> topBottomVelocityRPM;

    public ShooterIOFalcon500() {
        OrchestraContainer.getInstance().addMotor(topMotor);
        OrchestraContainer.getInstance().addMotor(bottomMotor);

        topMotor.getConfigurator().apply(ShooterConstants.kTopConfigs);
        topMotor.getConfigurator().apply(ShooterConstants.kTopConfigsHigh);

        bottomMotor.getConfigurator().apply(ShooterConstants.kBottomConfigs);
        bottomMotor.getConfigurator().apply(ShooterConstants.kBottomConfigsHigh);

        topBottomSupplyCurrent = List.of(topMotor.getSupplyCurrent(), bottomMotor.getSupplyCurrent());
        topBottomSupplyVoltage = List.of(topMotor.getSupplyVoltage(), bottomMotor.getSupplyVoltage());
        topBottomPosition = List.of(topMotor.getPosition(), bottomMotor.getPosition());
        topBottomVelocityRPM = List.of(topMotor.getVelocity(), bottomMotor.getVelocity());

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                topBottomSupplyCurrent.get(0),
                topBottomSupplyCurrent.get(1),
                topBottomSupplyVoltage.get(0),
                topBottomSupplyVoltage.get(1),
                topBottomPosition.get(0),
                topBottomPosition.get(1),
                topBottomVelocityRPM.get(0),
                topBottomVelocityRPM.get(1));

        topMotor.optimizeBusUtilization(1.0);
        bottomMotor.optimizeBusUtilization(1.0);

        topMotor.setInverted(true);
        bottomMotor.setInverted(false);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorConnected = BaseStatusSignal.refreshAll(
                topBottomSupplyCurrent.get(0),
                topBottomSupplyVoltage.get(0),
                topBottomPosition.get(0),
                topBottomVelocityRPM.get(0))
                .isOK();

        inputs.bottomMotorConnected = BaseStatusSignal.refreshAll(
                topBottomSupplyCurrent.get(1),
                topBottomSupplyVoltage.get(1),
                topBottomPosition.get(1),
                topBottomVelocityRPM.get(1))
                .isOK();

        inputs.topCurrent = topBottomSupplyCurrent.get(0).getValueAsDouble();
        inputs.bottomCurrent = topBottomSupplyCurrent.get(1).getValueAsDouble();

        inputs.topVoltage = topBottomSupplyVoltage.get(0).getValueAsDouble();
        inputs.bottomVoltage = topBottomSupplyVoltage.get(1).getValueAsDouble();

        inputs.topPosition = topBottomPosition.get(0).getValueAsDouble();
        inputs.bottomPosition = topBottomPosition.get(1).getValueAsDouble();

        inputs.topVelocity = topBottomVelocityRPM.get(0).getValueAsDouble();
        inputs.bottomVelocity = topBottomVelocityRPM.get(1).getValueAsDouble();
        inputs.sideVelocity = sideSetpoint;

        inputs.topSetpoint = topSetpoint;
        inputs.bottomSetpoint = bottomSetpoint;
        inputs.sideSetpoint = sideSetpoint;
    }

    @Override
    public void setTopBottomRPM(double RPM, double differential) {
        topMotor.setControl(voltageOut.withVelocity((RPM - differential) /
                60.0).withSlot(RPM > 2000 ? 1 : 0));
        bottomMotor.setControl(voltageOut.withVelocity((RPM + differential) /
                60.0).withSlot(RPM > 2000 ? 1 : 0));

        topSetpoint = (RPM - differential) / 60.0;
        bottomSetpoint = (RPM + differential) / 60.0;
    }

    @Override
    public void setSideRPM(double RPM) {
        sideSetpoint = RPM;
    }

    @Override
    public void setTopBottomVoltage(double voltage) {
        topMotor.setVoltage(voltage);
        bottomMotor.setVoltage(voltage);
    }

    @Override
    public void setSideVoltage(double voltage) {
    }
}
