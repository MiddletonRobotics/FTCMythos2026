package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.controller.wpilibcontroller.SimpleMotorFeedforward;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private DcMotorEx turretMotor;
    private DigitalChannel leftHomingSwitch;

    private Telemetry telemetry;

    public enum SystemState {
        TARGET_POSITION,

    }

    private PIDFController positionController;
    private SimpleMotorFeedforward frictionController;

    public static Turret instance;
    public static Turret getInstance(HardwareMap hMap, Telemetry telemetry) {
        if(instance == null) {
            instance = new Turret(hMap, telemetry);
        }

        return instance;
    }

    private double tx;
    private boolean hasTarget;

    private Turret(HardwareMap hMap, Telemetry telemetry) {
        turretMotor = hMap.get(DcMotorEx.class, TurretConstants.turretMotorID);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        positionController = new PIDFController(0.01,0,0,0);
        frictionController = new SimpleMotorFeedforward(0,0,0);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

    }

    public void setPosition(double degrees) {
        telemetry.addData("TurretPositionSetpoint", degrees);
        turretMotor.setPower(positionController.calculate(getCurrentPosition(), degrees) + frictionController.calculate(getCurrentVelocity()));
    }

    public boolean isAtSetpoint() {
        return positionController.atSetPoint();
    }

    public double getCurrentVelocity() {
        return turretMotor.getVelocity(AngleUnit.DEGREES);
    }

    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }
}
