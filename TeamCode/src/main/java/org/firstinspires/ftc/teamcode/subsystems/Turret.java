package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.library.math.GeometryUtilities;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private DcMotorEx turretMotor;
    private RevTouchSensor homingSwitch;

    private PIDFController primaryPositionController;
    private PIDFController secondaryPositionController;

    private Telemetry telemetry;

    public Turret(HardwareMap hMap, Telemetry telemetry) {
        turretMotor = hMap.get(DcMotorEx.class, TurretConstants.turretMotorID);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        homingSwitch = hMap.get(RevTouchSensor.class, TurretConstants.homingSwitchID);

        primaryPositionController = new PIDFController(TurretConstants.pP, TurretConstants.pI, TurretConstants.pD, TurretConstants.pF);
        secondaryPositionController = new PIDFController(TurretConstants.sP, TurretConstants.sI, TurretConstants.sD, TurretConstants.sF);
        primaryPositionController.setTolerance(3);
        secondaryPositionController.setTolerance(3);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData(TurretConstants.kSubsystemName + "Homing Switch Triggered?", isHomingTriggered());
        telemetry.addData(TurretConstants.kSubsystemName + "Current Open Loop", turretMotor.getPower());
        telemetry.addData(TurretConstants.kSubsystemName + "Current Position", getCurrentPosition());
    }

    public void setPosition(double degrees) {
        telemetry.addData(TurretConstants.kSubsystemName + "Setpoint Position", degrees);
        telemetry.addData(TurretConstants.kSubsystemName + "Primary Position Error", primaryPositionController.getPositionError());
        telemetry.addData(TurretConstants.kSubsystemName + "Primary At Setpoint?", primaryPositionController.atSetPoint());
        telemetry.addData(TurretConstants.kSubsystemName + "Secondary Position Error", primaryPositionController.getPositionError());
        telemetry.addData(TurretConstants.kSubsystemName + "Secondary At Setpoint?", isAtSetpoint());

        if(GlobalConstants.kTuningMode) {
            primaryPositionController.setPIDF(TurretConstants.pP, TurretConstants.pI, TurretConstants.pD, TurretConstants.pF);
            secondaryPositionController.setPIDF(TurretConstants.sP, TurretConstants.sI, TurretConstants.sD, TurretConstants.sF);
        }

        primaryPositionController.setSetPoint(degrees);
        secondaryPositionController.setSetPoint(degrees);

        if(Math.abs(primaryPositionController.getPositionError()) > TurretConstants.pidfSwitch) {
            turretMotor.setPower(primaryPositionController.calculate(getCurrentPosition(), degrees));
        } else {
            turretMotor.setPower(secondaryPositionController.calculate(getCurrentPosition(), degrees));
        }
    }

    public void setManualPower(double speed) {
        telemetry.addData(TurretConstants.kSubsystemName + "Setpoint Open Loop", speed);
        turretMotor.setPower(speed);
    }

    public void resetPosition() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isAtSetpoint() {
        return secondaryPositionController.atSetPoint();
    }

    public double getCurrentPosition() {
        return ((turretMotor.getCurrentPosition() / 537.7) / TurretConstants.turretGearRatio) * 360;
    }

    public boolean isHomingTriggered() {
        return homingSwitch.isPressed();
    }

    public double computeAngle(Pose2d robotPose, Pose2d targetPose, double turretOffsetX, double turretOffsetY) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();

        double turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
        double turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);

        double dx = targetPose.getX() - turretX;
        double dy = targetPose.getY() - turretY;

        double targetAngleGlobal = Math.atan2(dy, dx);
        double desiredTurretAngle = targetAngleGlobal - robotHeading;
        return GeometryUtilities.normalizeAngle(desiredTurretAngle);
    }
}
