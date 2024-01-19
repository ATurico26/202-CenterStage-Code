package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueFar", group = "Iterative Opmode")
public class BlueFar extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);


        boolean left = false;
        boolean right = false;

        waitForStart();

        //Close claw
        robot.Claw.setPosition(0);

        sleep(500);



        // Find where the team object is, move, and place pixel

        double[] objectLocation = robot.teamObjectPosition(11, 10);

        if (Math.round(objectLocation[0]) == 0) {
            telemetry.addLine("Object at left");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();
            left = true;

            sleep(500);


        }
        else if (Math.round(objectLocation[0]) == 2) {
            telemetry.addLine("Object at right");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();
            right = true;

            sleep(500);


        }
        else {
            telemetry.addLine("Object at middle");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            sleep(500);


        }

        sleep(250);


        sleep(500);


        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
