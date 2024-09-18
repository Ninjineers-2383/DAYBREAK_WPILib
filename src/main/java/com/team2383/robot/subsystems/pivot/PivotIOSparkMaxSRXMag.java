package com.team2383.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PivotIOSparkMaxSRXMag implements PivotIO {
    private final CANSparkMax motorLeader;
    private final SparkPIDController leaderPID;

    private double currentPositionRad = 0.0;

    private double offset = 0;

    private boolean hasReset = false;

    private double desiredRot = 0.0;
    private double desiredVel = 0.0;

    private double kS = PivotConstants.kGains.kS();
    private double kV = PivotConstants.kGains.kV();
    private double kG = PivotConstants.kGains.kG();

    public PivotIOSparkMaxSRXMag() {
        motorLeader = new CANSparkMax(PivotConstants.kLeftMotorID, MotorType.kBrushless);

        motorLeader.setInverted(true);

        leaderPID = motorLeader.getPIDController();

        motorLeader.getEncoder().setPositionConversionFactor(PivotConstants.kPivotMotorGearRatio);

        leaderPID.setP(PivotConstants.kGains.kP());
        leaderPID.setI(PivotConstants.kGains.kI());
        leaderPID.setD(PivotConstants.kGains.kD());
    }

    public void updateInputs(PivotIOInputs inputs) {
        if (!hasReset && motorLeader.getEncoder().getPosition() != 0.0) {
            motorLeader.getEncoder().setPosition(0.0);
        } else {
            hasReset = true;
        }

        inputs.leftMotorConnected = true;
        inputs.rightMotorConnected = true;

        inputs.rotorPositionRot = motorLeader.getEncoder().getPosition();
        inputs.desiredPositionRot = desiredRot;
        currentPositionRad = inputs.rotorPositionRot * 2 * Math.PI;

        inputs.velocityRotPerSec = motorLeader.getEncoder().getVelocity() * 60.0;
        inputs.desiredVelocityRotPerSec = desiredVel;

        inputs.appliedVolts = new double[] { motorLeader.get() * 12.0 };

        motorLeader.getPIDController().setReference(desiredRot + offset, CANSparkBase.ControlType.kPosition, 0,
                (desiredVel * kV) + kS + Math.sin(currentPositionRad) * kG);
    }

    @Override
    public void setAngleRot(double angleRot, double velocityRotPerSec, PivotSubsystem.LashState lashState) {
        desiredRot = angleRot;
        desiredVel = velocityRotPerSec;

    }

    @Override
    public void setVoltage(double volts) {
        motorLeader.setVoltage(volts);
    }

    @Override
    public void disable() {
        motorLeader.disable();
    }

    @Override
    public void setPIDController(double kP, double kI, double kD) {
        leaderPID.setP(kP);
        leaderPID.setI(kI);
        leaderPID.setD(kD);
    }

    @Override
    public void setFeedforward(double kS, double kV, double kA, double kG, double kSpring) {
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;

        System.out.println("Pivot Feedforward changes");
    }
}
