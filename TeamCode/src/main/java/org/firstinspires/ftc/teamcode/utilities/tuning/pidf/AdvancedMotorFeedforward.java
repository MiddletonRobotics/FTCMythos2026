package org.firstinspires.ftc.teamcode.utilities.tuning.pidf;

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

    @Override
    public void init() {
        motor = new MotorEx(hardwareMap, "");
        encoder = new MotorEx(hardwareMap, "").encoder;

        motor.setInverted(false);
        encoder.reset();
        power = 0.0;
    }

    @Override
    public void init_loop() {
        if(startButtonPressed) {
            telemetry.addLine("Square (X): Increase power by 0.01");
            telemetry.addLine("Triangle (Y): Decrease power by 0.01");
            telemetry.addLine("Cross (A): Continue / output final results (need both save slots full)");
            telemetry.addLine("Left Bumper: Save slot 1 (-1 represents no data saved)");
            telemetry.addLine("Right Bumper: Save slot 2 (-1 represents no data saved)");
            telemetry.update();
        } else {
            telemetry.addLine("This will run the motor specified initially at 0 power.");
            telemetry.addLine("Make sure the motor is able to rotate multiple times without damaging the system.");
            telemetry.addLine("Increase power until the elevator barely moves up, save it, then decrease power until it barely moves down, and save it.");
            telemetry.addLine("Press the start button to bring up the controls");
            telemetry.update();

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

                telemetry.addData("Calculated kG Value: ", kG);
                telemetry.addData("Calculated kS Value: ", kS);
                telemetry.update();

                isFinishedTuning = true;
            }
        } else {
            motor.set(power);

            telemetry.addData("Motor Power: ", power);
            telemetry.addData("Current Position: ", encoder.getPosition());
            telemetry.addData("Current Velocity: ", encoder.getCorrectedVelocity());
            telemetry.addLine("\n");
            telemetry.addData("Save Slot 1: ", saveSlot1);
            telemetry.addData("Save Slot 2: ", saveSlot2);
            telemetry.update();
        }

        aButtonPreviousState = gamepad1.a;
        xButtonPreviousState = gamepad1.square;
        yButtonPreviousState = gamepad1.triangle;
        leftBumperButtonPreviousState = gamepad1.left_bumper;
        rightBumperButtonPreviousState = gamepad1.right_bumper;
    }
}