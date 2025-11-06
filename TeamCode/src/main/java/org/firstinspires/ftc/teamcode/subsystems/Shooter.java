package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.lights.Headlight;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Shooter extends SubsystemBase {
    private ServoEx f;
    private MotorEx leftShooter, rightShooter;
    private PIDFController b, s;

    private double t = 0;
    public static double bp = 0.03, bd = 0.0, bf = 0.0, sp = 0.01, sd = 0.0001, sf = 0.0;

    public static double pSwitch = 50;
    private boolean activated = true, targetSpotted = false;

    public static double close = 1200;
    public static double far = 2000;
    public static double flipUp = 0.3;
    public static double flipDown = 0.5;

    private static Shooter instance;
    public static synchronized Shooter getInstance(HardwareMap hMap) {
        if(instance == null) {
            instance = new Shooter(hMap);
        }

        return instance;
    }

    private Shooter(HardwareMap hardwareMap) {
        b = new PIDFController(bp, 0, bd, bf);
        s = new PIDFController(sp, 0, sd, sf);
        leftShooter = new MotorEx(hardwareMap, "sl");
        rightShooter = new MotorEx(hardwareMap, "sr");
        f = new ServoEx(hardwareMap, "f");

        rightShooter.setInverted(true);
    }

    /** in/s */
    public double getTarget() {
        return t;
    }

    /** in/s */
    public double getVelocity() {
        return leftShooter.getVelocity();
    }

    public void setPower(double p) {
        leftShooter.set(p);
        rightShooter.set(p);
    }

    public void toggle() {
        activated = !activated;
        if (!activated) setPower(0);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }

    public void far() {
        setTarget(far);
        on();
    }

    public void close() {
        setTarget(close);
        on();
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    @Override
    public void periodic() {
        b.setPIDF(bp, 0, bd, bf);
        s.setPIDF(sp, 0, sd, sf);

        if (activated) {
            if (Math.abs(getTarget() - getVelocity()) < pSwitch) {
                setPower(s.calculate(getVelocity(), getTarget()));
            } else {
                setPower(b.calculate(getVelocity(), getTarget()));
            }
        }
    }

    public void up() {
        f.set(flipUp);
    }

    public void down() {
        f.set(flipDown);
    }

    public void flip() {
        if (f.getRawPosition() == flipDown)
            up();
        else
            down();
    }

    public void targetSpotted(boolean spotted) {
        targetSpotted = spotted;
    }

    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 100;
    }

    public void forDistance(double distance) {
        setTarget((6.13992 * distance) + 858.51272);
    }

}