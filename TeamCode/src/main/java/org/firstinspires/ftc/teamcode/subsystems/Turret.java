package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private DcMotorEx turretMotor;
    private RevTouchSensor homingSwitch;

    private PIDFController primaryPositionController;
    private PIDFController secondaryPositionController;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Turret(HardwareMap hMap, TelemetryManager telemetryM) {
        turretMotor = hMap.get(DcMotorEx.class, TurretConstants.turretMotorID);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        homingSwitch = hMap.get(RevTouchSensor.class, TurretConstants.homingSwitchID);

        primaryPositionController = new PIDFController(TurretConstants.pP, TurretConstants.pI, TurretConstants.pD, TurretConstants.pF);
        secondaryPositionController = new PIDFController(TurretConstants.sP, TurretConstants.sI, TurretConstants.sD, TurretConstants.sF);
        primaryPositionController.setTolerance(3);
        secondaryPositionController.setTolerance(3);

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData(TurretConstants.kSubsystemName + "Homing Switch Triggered?", isHomingTriggered());
        telemetryM.addData(TurretConstants.kSubsystemName + "Current Open Loop", turretMotor.getPower());
        telemetryM.addData(TurretConstants.kSubsystemName + "Current Position", getCurrentPosition());

        if(isHomingTriggered()) {
            resetPosition();
        }
    }

    public void setPosition(double radians) {
        double friction;
        telemetryM.addData(TurretConstants.kSubsystemName + "Setpoint Position", radians);
        telemetryM.addData(TurretConstants.kSubsystemName + "Primary Position Error", primaryPositionController.getPositionError());
        telemetryM.addData(TurretConstants.kSubsystemName + "Primary At Setpoint?", primaryPositionController.atSetPoint());
        telemetryM.addData(TurretConstants.kSubsystemName + "Secondary Position Error", primaryPositionController.getPositionError());
        telemetryM.addData(TurretConstants.kSubsystemName + "Secondary At Setpoint?", isAtSetpoint());

        if(GlobalConstants.kTuningMode) {
            primaryPositionController.setPIDF(TurretConstants.pP, TurretConstants.pI, TurretConstants.pD, TurretConstants.pF);
            secondaryPositionController.setPIDF(TurretConstants.sP, TurretConstants.sI, TurretConstants.sD, TurretConstants.sF);
        }

        primaryPositionController.setSetPoint(radians);
        secondaryPositionController.setSetPoint(radians);

        if(primaryPositionController.getPositionError() < 0) {
            friction = -0.11;
        } else {
            friction = 0.11;
        }

        if(Math.abs(primaryPositionController.getPositionError()) > TurretConstants.pidfSwitch) {
            turretMotor.setPower(primaryPositionController.calculate(getCurrentPosition(), radians) + friction);
        } else {
            turretMotor.setPower(secondaryPositionController.calculate(getCurrentPosition(), radians) + friction);
        }
    }

    public void setManualPower(double speed) {
        telemetryM.addData(TurretConstants.kSubsystemName + "Setpoint Open Loop", speed);
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
        return Math.toRadians(GeometryUtilities.normalizeAngle(((turretMotor.getCurrentPosition() / 537.7) / TurretConstants.turretGearRatio) * 360));
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
        return AngleUnit.normalizeRadians(desiredTurretAngle);
    }
}
