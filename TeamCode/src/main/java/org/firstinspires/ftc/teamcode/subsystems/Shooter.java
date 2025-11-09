package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.lights.Headlight;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private ServoEx hoodServo;
    private MotorEx shooterMotor;

    public enum SystemState {
        IDLE,
        RUNNING,
        EXHAUSTING
    }

    public enum WantedState {
        IDLE,
        RUNNING,
        EXHAUSTING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private PIDFController controller;

    private static Shooter instance;
    public static synchronized Shooter getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Shooter(hMap);
        }

        return instance;
    }

    private Shooter(HardwareMap hardwareMap) {
        shooterMotor = new MotorEx(hardwareMap, ShooterConstants.shooterMotorID, Motor.GoBILDA.BARE);
        controller = new PIDFController(0.01, 0, 0.0, 0.0);
    }
}