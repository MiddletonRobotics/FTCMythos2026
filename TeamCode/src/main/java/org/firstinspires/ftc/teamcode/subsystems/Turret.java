package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.library.hardware.motors.Motor;
import org.firstinspires.ftc.library.hardware.motors.MotorEx;
import org.firstinspires.ftc.library.math.GeometryUtilities;
import org.firstinspires.ftc.library.math.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private DcMotorEx turretMotor;
    private DigitalChannel leftHomingSwitch;

    private PIDFController positionController;
    private SimpleMotorFeedforward frictionController;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Turret(HardwareMap hMap, TelemetryManager telemetryManager) {
        turretMotor = hMap.get(DcMotorEx.class, TurretConstants.turretMotorID);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        positionController = new PIDFController(TurretConstants.P, TurretConstants.I, TurretConstants.D, 0);
        frictionController = new SimpleMotorFeedforward(0.08,0,0);

        positionController.setTolerance(10);

        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {
        telemetryManager.addData(TurretConstants.kSubsystemName + "Current Position", getCurrentPosition());
    }

    public void setPosition(double ticks) {
        telemetryManager.addData(TurretConstants.kSubsystemName + "Setpoint Position", ticks);
        telemetryManager.addData(TurretConstants.kSubsystemName + "Position Error", positionController.getPositionError());
        telemetryManager.addData(TurretConstants.kSubsystemName + "At Setpoint?", isAtSetpoint());

        positionController.setPIDF(TurretConstants.P, TurretConstants.I, TurretConstants.D, 0);
        turretMotor.setPower(positionController.calculate(getCurrentPosition(), ticks) + frictionController.calculate(getCurrentVelocity()));
    }

    public void setManualPower(double power) {
        turretMotor.setPower(power);
    }

    public boolean isAtSetpoint() {
        return positionController.atSetPoint();
    }

    public double getCurrentVelocity() {
        return ((turretMotor.getVelocity() / TurretConstants.turretGearRatio) / 384.5) * 360;
    }

    public double getCurrentPosition() {
        return ((turretMotor.getCurrentPosition() / TurretConstants.turretGearRatio) / 384.5) * 360;
    }

    public static double computeAimAngle(Pose2d robotPose, Pose2d tagPose, double turretOffsetX, double turretOffsetY, double turretForwardAngle) {

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();   // radians

        // Compute turret global position
        double turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
        double turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);

        double turretFacingGlobal = robotHeading + turretForwardAngle;

        // Vector turret → tag
        double dx = tagPose.getX() - turretX;
        double dy = tagPose.getY() - turretY;

        // Angle turret should face globally
        double targetAngleGlobal = Math.atan2(dy, dx);

        double desiredTurretAngle = targetAngleGlobal - turretFacingGlobal;

        return GeometryUtilities.normalizeAngle(desiredTurretAngle);
    }
}
