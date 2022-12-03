package org.firstinspires.ftc.teamcode.whiteout;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "whiteoutBotManual", group = "Linear Opmode")
//@Disabled
public class whiteoutBotManual extends whiteoutBotCommon {
    private float leftX, leftY, rightZ;
    private double motor_power = 0.3;

    @Override
    public void runOpMode() {
        initHW();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.dtDriver.checkGamePad(gamepad1);
            robot.inTakeDriver.checkGamePad(gamepad1);
            robot.slideDriver.checkGamePad(gamepad2);
            robot.slideDriver.checkGamePadX(gamepad1); //only to rotate the arm

            //robot.armDriver.checkGamePad(gamepad2);
            robot.crDriver.checkGamePad(gamepad2);
            robot.frDriver.checkGamePad(gamepad1);

            idle();


        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }
}

