package org.firstinspires.ftc.teamcode.utilities.tuning.pidf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.library.hardware.motors.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Disabled
@TeleOp(name="AdvancedMotorFeedforward")
public class AdvancedMotorFeedforward extends OpMode {
    private MotorEx motor;
    private MotorEx.Encoder encoder;

    private double power;
    private double saveSlot1, saveSlot2 = -1;

    private boolean aButtonPreviousState = false;
    private boolean xButtonPreviousState, yButtonPreviousState = false;
    private boolean leftBumperButtonPreviousState, rightBumperButtonPreviousState = false;
    private boolean isFinishedTuning = false;
    private boolean startButtonPressed = false;

    private Telemetry telemetryA;

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetryA, FtcDashboard.getInstance().getTelemetry());

        motor = new MotorEx(hardwareMap, "");
        encoder = new MotorEx(hardwareMap, "").encoder;

        motor.setInverted(false);
        encoder.reset();
        power = 0.0;
    }

    @Override
    public void init_loop() {
        if(startButtonPressed) {
            telemetryA.addLine("Square (X): Increase power by 0.01");
            telemetryA.addLine("Triangle (Y): Decrease power by 0.01");
            telemetryA.addLine("Cross (A): Continue / output final results (need both save slots full)");
            telemetryA.addLine("Left Bumper: Save slot 1 (-1 represents no data saved)");
            telemetryA.addLine("Right Bumper: Save slot 2 (-1 represents no data saved)");
            telemetryA.update();
        } else {
            telemetryA.addLine("This will run the motor specified initially at 0 power.");
            telemetryA.addLine("Make sure the motor is able to rotate multiple times without damaging the system.");
            telemetryA.addLine("Increase power until the elevator barely moves up, save it, then decrease power until it barely moves down, and save it.");
            telemetryA.addLine("Press the start button to bring up the controls");
            telemetryA.update();

            if(gamepad1.start) {
                startButtonPressed = true;
            }
        }
    }

    @Override
    public void loop() {
        if (gamepad1.square && !xButtonPreviousState) {
            if (power == 1) {
                power -= 0;
            } else {
                power += 0.01;
            }
        } else if (gamepad1.triangle && !yButtonPreviousState) {
            if (power == 0) {
                power -= 0;
            } else {
                power -= 0.01;
            }
        }

        if(gamepad1.left_bumper && !leftBumperButtonPreviousState) {
            saveSlot1 = power;
        } else if(gamepad1.right_bumper && !rightBumperButtonPreviousState) {
            saveSlot2 = power;
        }

        if(gamepad1.a && !aButtonPreviousState || isFinishedTuning) {
            if(saveSlot1 != -1 && saveSlot2 != -1) {
                double min = Math.min(saveSlot1, saveSlot2);
                double max = Math.max(saveSlot1, saveSlot2);

                double kG = (max + min) / 2;
                double kS = (max - min) / 2;

                telemetryA.addData("Calculated kG Value: ", kG);
                telemetryA.addData("Calculated kS Value: ", kS);
                telemetryA.update();

                isFinishedTuning = true;
            }
        } else {
            motor.set(power);

            telemetryA.addData("Motor Power: ", power);
            telemetryA.addData("Current Position: ", encoder.getPosition());
            telemetryA.addData("Current Velocity: ", encoder.getCorrectedVelocity());
            telemetryA.addLine("\n");
            telemetryA.addData("Save Slot 1: ", saveSlot1);
            telemetryA.addData("Save Slot 2: ", saveSlot2);
            telemetryA.update();
        }

        aButtonPreviousState = gamepad1.a;
        xButtonPreviousState = gamepad1.square;
        yButtonPreviousState = gamepad1.triangle;
        leftBumperButtonPreviousState = gamepad1.left_bumper;
        rightBumperButtonPreviousState = gamepad1.right_bumper;
    }
}