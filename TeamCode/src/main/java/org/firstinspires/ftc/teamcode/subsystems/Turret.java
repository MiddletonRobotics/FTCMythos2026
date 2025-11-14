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
            case TARGET_POSITION:
            case MANUAL:
        }
    }

    public void setTargetPosition(double targetPosition) {
        positionController.setSetPoint(targetPosition);
        wantedState = WantedState.TARGET_POSITION;
    }
}
