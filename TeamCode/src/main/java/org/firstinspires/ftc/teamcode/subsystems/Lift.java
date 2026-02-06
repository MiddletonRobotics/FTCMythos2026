package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.library.command.SubsystemBase;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.library.hardware.AnalogEncoder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;

import lombok.Getter;
import lombok.Setter;

public class Lift  extends SubsystemBase {
    private final CRServo liftServo;
    private final AnalogEncoder liftEncoder;
    private final RevTouchSensor homingSwitch;

    private PIDFController positonController;

    @Getter
    @Setter
    private double relativePosition;

    private double lastAbsolutePosition;
    private boolean isRelativeInitialized;

    @Getter
    private boolean isMechanismHomed;

    @Getter
    private boolean isCurrentlyHoming;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public Lift(HardwareMap hMap, TelemetryManager telemetryM) {
        liftServo = hMap.get(CRServo.class, LiftConstants.kLiftServoID);
        liftServo.setDirection(DcMotorSimple.Direction.FORWARD);

        liftEncoder = new AnalogEncoder(hMap, LiftConstants.kLiftServoEncoderID, 3.3, AngleUnit.RADIANS);
        liftEncoder.setReversed(false);

        positonController = new PIDFController(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD, LiftConstants.kF);

        homingSwitch = hMap.get(RevTouchSensor.class, LiftConstants.kLiftHomingSwitchID);

        relativePosition = 0.0;
        lastAbsolutePosition = liftEncoder.getCurrentPosition();
        isRelativeInitialized = true;

        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        telemetryM.addData(LiftConstants.kSubsystemName + "Servo Encoder Absolute Position", getAbsolutePosition());
        telemetryM.addData(LiftConstants.kSubsystemName + "Servo Encoder Relative Position", getRelativePosition());
        telemetryM.addData(LiftConstants.kSubsystemName + "Servo Power", liftServo.getPower());
        telemetryM.addData(LiftConstants.kSubsystemName + "Homing Switch Triggered?", isHomingSwitchTriggered());

        updateRelativePosition();
    }

    public void onInitialization() {
        resetToZero();

        if(isHomingSwitchTriggered()) {
            stopHoming();
        }
    }

    public void setPosition(double positionRotations) {
        double motorPower = positonController.calculate(getRelativePosition(), positionRotations);
        setPower(motorPower);
    }

    public boolean isAtSetpoint() {
        return positonController.atSetPoint();
    }

    private void updateRelativePosition() {
        if (!isRelativeInitialized) {
            lastAbsolutePosition = liftEncoder.getCurrentPosition();
            isRelativeInitialized = true;
            return;
        }

        double currentAbsolute = liftEncoder.getCurrentPosition();
        double delta = currentAbsolute - lastAbsolutePosition;

        if (Math.abs(delta) > Math.PI) {
            delta = delta - Math.signum(delta) * 2 * Math.PI;
        }

        relativePosition += delta;
        lastAbsolutePosition = currentAbsolute;
    }

    public void resetToZero() {
        setPower(-0.5);
        isMechanismHomed = false;
    }

    public void stopHoming() {
        setPower(0);
        resetRelativePosition();

        isMechanismHomed = false;
        isCurrentlyHoming = false;
    }

    public void setPower(final double power) {
        liftServo.setPower(power);
    }

    public double getAbsolutePosition() {
        return liftEncoder.getCurrentPosition();
    }

    public boolean isHomingSwitchTriggered() {
        return homingSwitch.isPressed();
    }

    public void resetRelativePosition() {
        relativePosition = 0.0;
        lastAbsolutePosition = liftEncoder.getCurrentPosition();
    }
}
