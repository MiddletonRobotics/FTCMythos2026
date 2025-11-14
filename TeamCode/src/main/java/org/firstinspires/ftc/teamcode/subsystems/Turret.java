package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private DcMotorEx turretMotor;
    private DigitalChannel leftHomingSwitch;

    public enum SystemState {
        IDLE,
        HOME,
        FINDING_POSITION,
        RELOCALIZING,
        TARGET_POSITION,
        MANUAL
    }

    public enum WantedState {
        IDLE,
        HOME,
        FINDING_POSITION,
        RELOCALIZING,
        TARGET_POSITION,
        MANUAL
    }

    private PIDFController positionController;
    private SimpleMotorFeedforward frictionController;

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public static Turret instance;
    public static synchronized Turret getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Turret(hMap);
        }

        return instance;
    }

    private double tx;
    private boolean hasTarget;

    private Turret(HardwareMap hMap) {
        turretMotor = hMap.get(DcMotorEx.class, TurretConstants.turretMotorID);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        positionController = new PIDFController(0.01,0,0,0);
        frictionController = new SimpleMotorFeedforward(0,0,0);
    }

    @Override
    public void periodic() {
        systemState = handleTransition();
        applyStates();
    }

    private SystemState handleTransition() {
        double angle = getAngle();

        if (angle > 360 && systemState != SystemState.RELOCALIZING) {
            return SystemState.RELOCALIZING;
        }

        switch(wantedState) {
            case IDLE:
                turretMotor.setPower(0);
                systemState =  SystemState.IDLE;
            case HOME:
                systemState = SystemState.HOME;
            case FINDING_POSITION:
                systemState = SystemState.FINDING_POSITION;
            case RELOCALIZING:
                systemState = SystemState.RELOCALIZING;
            case TARGET_POSITION:
                systemState = SystemState.TARGET_POSITION;
            case MANUAL:
                systemState = SystemState.MANUAL;
        }

        return systemState;
    }

    public void applyStates() {
        switch(systemState) {
            case IDLE:
            case HOME:
            case FINDING_POSITION:
            case RELOCALIZING:
                relocalize();
            case TARGET_POSITION:
                if (hasTarget) {
                    aimWithVision();
                } else {
                    turretMotor.setPower(0);
                }
            case MANUAL:
        }
    }

    private void aimWithVision() {
        double currentAngle = getAngle();
        double targetAngle = currentAngle + tx;

        // Clamp to safe range
        targetAngle = Math.max(0, Math.min(360, targetAngle));

        double power = positionController.calculate(currentAngle, targetAngle);
        turretMotor.setPower(power);
    }

    private void relocalize() {
        double currentAngle = getAngle();
        double target = 0;

        double power = positionController.calculate(currentAngle, target);
        turretMotor.setPower(power);

        // When close enough, stop and go to idle or tracking
        if (Math.abs(currentAngle - 0) < 2.0) {  // 2 degree tolerance
            turretMotor.setPower(0);
            wantedState = WantedState.TARGET_POSITION; // automatically resume
        }
    }

    public void setManualPowerControl(double power) {
        wantedState = WantedState.MANUAL;
        turretMotor.setPower(power);
    }

    public void setLimelightData(double tx, boolean hasTarget) {
        this.tx = tx;
        this.hasTarget = hasTarget;
    }

    public void startTracking() {
        wantedState = WantedState.TARGET_POSITION;
    }

    public void forceReturnToZero() {
        wantedState = WantedState.RELOCALIZING;
    }
    private double getAngle() {
        return (turretMotor.getCurrentPosition() * (360.0 / 1024)) / 13;
    }
}
