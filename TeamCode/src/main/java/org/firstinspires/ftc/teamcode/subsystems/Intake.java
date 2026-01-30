package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private DcMotorEx frontIntakeMotor;
    private DcMotorEx rearIntakeMotor;

    private PIDFController velocityPIDFController;
    private SimpleMotorFeedforward velocityFeedforward;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Intake(HardwareMap hMap, TelemetryManager telemetryM) {
        frontIntakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.frontIntakeMotorID);
        rearIntakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.rearIntakeMotorID);

        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPIDFController = new PIDFController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kF);
        velocityFeedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "Current Velocity", getFrontVelocity());
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "Current Open Loop", frontIntakeMotor.getPower());
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "Current Velocity", getRearVelocity());
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "Current Open Loop", rearIntakeMotor.getPower());
    }

    public void setVelocitySetpoint(double targetRPM) {
        telemetryM.addData(IntakeConstants.kSubsystemName + "Setpoint Velocity", targetRPM);
        telemetryM.addData(IntakeConstants.kSubsystemName + "Velocity Error", velocityPIDFController.getPositionError());
        telemetryM.addData(IntakeConstants.kSubsystemName + "At Setpoint?", velocityPIDFController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            velocityPIDFController.setPIDF(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kF);
            velocityFeedforward.setCoefficient(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
        }

        frontIntakeMotor.setPower(velocityPIDFController.calculate(getFrontVelocity(), targetRPM) + velocityFeedforward.calculate(targetRPM));
        rearIntakeMotor.setPower(velocityPIDFController.calculate(getRearVelocity(), targetRPM) + velocityFeedforward.calculate(targetRPM));
    }

    public void setFrontMotorOpenLoop(double speed) {
        telemetryM.addData("Front" + IntakeConstants.kSubsystemName + "Setpoint Open Loop", speed);
        frontIntakeMotor.setPower(speed);
    }

    public void setRearMotorOpenLoop(double speed) {
        telemetryM.addData("Rear" + IntakeConstants.kSubsystemName + "Setpoint Open Loop", speed);
        rearIntakeMotor.setPower(speed);
    }


    public void setOpenLoopSetpoint(double speed) {
        setFrontMotorOpenLoop(speed);
        setRearMotorOpenLoop(speed);
    }

    public void setEqualOpenLoopSetpoint(double rearSpeed) {
        double frontSpeed = rearSpeed;

        setFrontMotorOpenLoop(frontSpeed);
        setRearMotorOpenLoop(rearSpeed);
    }

    public double getFrontVelocity() {
        return ((frontIntakeMotor.getVelocity() / IntakeConstants.intakeMotorCPR) * IntakeConstants.frontIntakeRatio) * 60;
    }

    public double getRearVelocity() {
        return ((rearIntakeMotor.getVelocity() / IntakeConstants.intakeMotorCPR) * IntakeConstants.rearIntakeRatio) * 60;
    }
}
