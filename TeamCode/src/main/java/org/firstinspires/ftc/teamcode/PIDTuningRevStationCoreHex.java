package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="StationPIDTuning", group="Station")
public class PIDTuningRevStationCoreHex extends LinearOpMode {

    HardwareRevStationCoreHex robot       = new HardwareRevStationCoreHex();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double max_V = 0;  // going to be 640 for station with Core Hex motor
        double F = 0; // F = 32767 / max_V = 51.2
        double P = 0; // P = 0.1 * F = 5.1
        double I = 0; // I = 0.1 * P = 0.51
        // D = 0
        // For position PIDF: P = 5.0

//        while (opModeIsActive() && !isStopRequested()) {
        while (opModeIsActive()) {
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            double vL = robot.leftDrive.getVelocity();
            double vR = robot.rightDrive.getVelocity();
            double min_motor_V = Math.min(vL, vR);
            if (min_motor_V > max_V) {max_V = min_motor_V;}
            F = 32767/max_V;
            P = 0.1 * F;
            I = 0.1 * P;
            telemetry.addData("max V", max_V);
            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("F", F);
            telemetry.update();
        }
    }

}

