package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RedFar", group = "Iterative Opmode")
public class RedFar extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);


        boolean middle = false;
        boolean right = false;

        waitForStart();

        //Close claw
        robot.Claw.setPosition(0);

        sleep(500);

        // Find where the team object is, move, and place pixel
        if (robot.LeftSensor.getDistance(DistanceUnit.INCH) < 35) {
            telemetry.addLine("Object at middle");
            telemetry.update();
            middle = true;

            sleep(500);


        }
        else if (robot.RightSensor.getDistance(DistanceUnit.INCH) < 35 && robot.RightSensor.getDistance(DistanceUnit.INCH) > 19.5) {
            telemetry.addLine("Object at right");
            telemetry.update();
            right = true;

            sleep(500);


        }
        else {
            telemetry.addLine("Object at left");
            telemetry.update();

            sleep(500);


        }

        sleep(250);


        sleep(500);


        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
