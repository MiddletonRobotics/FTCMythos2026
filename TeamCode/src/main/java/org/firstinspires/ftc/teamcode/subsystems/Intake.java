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

    private PIDFController firstRollerVelocityController;
    private SimpleMotorFeedforward firstRollerVelocityFeedForward;

    private PIDFController secondRollerVelocityController;
    private SimpleMotorFeedforward secondRollerVelocityFeedForward;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Intake(HardwareMap hMap, TelemetryManager telemetryM) {
        frontIntakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.frontIntakeMotorID);
        rearIntakeMotor = hMap.get(DcMotorEx.class, IntakeConstants.rearIntakeMotorID);

        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        firstRollerVelocityController = new PIDFController(IntakeConstants.fP, IntakeConstants.fI, IntakeConstants.fD, IntakeConstants.fF);
        firstRollerVelocityFeedForward = new SimpleMotorFeedforward(IntakeConstants.fS, IntakeConstants.fV, IntakeConstants.fA);

        secondRollerVelocityController = new PIDFController(IntakeConstants.sP, IntakeConstants.sI, IntakeConstants.sD, IntakeConstants.sF);
        secondRollerVelocityFeedForward = new SimpleMotorFeedforward(IntakeConstants.sS, IntakeConstants.sV, IntakeConstants.sA);

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "Current Velocity", getFrontVelocity());
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "Current Open Loop", frontIntakeMotor.getPower());
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "Current Velocity", getRearVelocity());
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "Current Open Loop", rearIntakeMotor.getPower());
    }

    public void setFrontVelocitySetpoint(double targetRPM) {
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "Setpoint Velocity", targetRPM);
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "Velocity Error", firstRollerVelocityController.getPositionError());
        telemetryM.addData("Front " + IntakeConstants.kSubsystemName + "At Setpoint?", firstRollerVelocityController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            firstRollerVelocityController.setPIDF(IntakeConstants.fP, IntakeConstants.fI, IntakeConstants.fD, IntakeConstants.fF);
            firstRollerVelocityFeedForward.setCoefficient(IntakeConstants.fS, IntakeConstants.fV, IntakeConstants.fA);
        }

        frontIntakeMotor.setPower(firstRollerVelocityController.calculate(getFrontVelocity(), targetRPM) + firstRollerVelocityFeedForward.calculate(targetRPM));
    }

    public void setRearVelocitySetpoint(double targetRPM) {
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "Setpoint Velocity", targetRPM);
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "Velocity Error", firstRollerVelocityController.getPositionError());
        telemetryM.addData("Rear " + IntakeConstants.kSubsystemName + "At Setpoint?", firstRollerVelocityController.atSetPoint());

        if(GlobalConstants.kTuningMode) {
            secondRollerVelocityController.setPIDF(IntakeConstants.sP, IntakeConstants.sI, IntakeConstants.sD, IntakeConstants.sF);
            secondRollerVelocityFeedForward.setCoefficient(IntakeConstants.sS, IntakeConstants.sV, IntakeConstants.sA);
        }

        rearIntakeMotor.setPower(secondRollerVelocityController.calculate(getFrontVelocity(), targetRPM) + secondRollerVelocityFeedForward.calculate(targetRPM));
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
        double frontSpeed = rearSpeed / IntakeConstants.frontIntakeRatio;

        setFrontMotorOpenLoop(frontSpeed);
        setRearMotorOpenLoop(rearSpeed);
    }

    public double getFrontVelocity() {
        return ((frontIntakeMotor.getVelocity() / IntakeConstants.kFrontIntakeMotorCPR) * IntakeConstants.frontIntakeRatio) * 60;
    }

    public double getRearVelocity() {
        return ((rearIntakeMotor.getVelocity() / IntakeConstants.kRearIntakeMotorCPR) * IntakeConstants.rearIntakeRatio) * 60;
    }
}
