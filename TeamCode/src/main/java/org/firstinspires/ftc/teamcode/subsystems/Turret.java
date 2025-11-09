package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.library.command.Command;
import org.firstinspires.ftc.library.command.Commands;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class Turret {
    private MotorEx turretMotor;
    private Motor.Encoder turretEncoder;
    private DigitalChannel leftHomingSwitch;
    private DigitalChannel rightHomingSwitch;

    public enum SystemState {
        IDLE,
        FINDING_POSITION,
        RELOCALIZING,
        TARGET_POSITION
    }

    public enum WantedState {
        IDLE,
        FINDING_POSITION,
        RELOCALIZING,
        TARGET_POSITION
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

    private Turret(HardwareMap hMap) {
        turretMotor = new MotorEx(hMap, TurretConstants.turretMotorID, Motor.GoBILDA.BARE);
        turretMotor.setInverted(true);

        positionController = new PIDFController(0.01,0,0,0);
        frictionController = new SimpleMotorFeedforward(0,0,0);

        turretEncoder = turretMotor.encoder;
    }

    public void setTargetPosition(double targetPosition) {
        positionController.setSetPoint(targetPosition);
        wantedState = WantedState.TARGET_POSITION;
    }
}
