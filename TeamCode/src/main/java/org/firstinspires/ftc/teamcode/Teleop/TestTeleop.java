package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;
//idiotproofing marked by //~comment
@TeleOp(name = "TestTeleop")

    //~this is a LinearOpMode
public class TestTeleop extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .8; //~speed limiter
        int rightPosition = 0; //~right and left lift motor targets
        int leftPosition = 0;
        int noU = -8000;
        int pos = 2;
        double rDiff = .5; //~diff zeros
        double lDiff = .5;
        int[] positions; 
        //~these are drivetrain motors, mode toggle is self-explanatory i hope
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /// init motor pos and encoder
        robot.rightLift.setTargetPosition(0);
        robot.leftLift.setTargetPosition(0);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //~telemetry is the printout in the DH
        telemetry.addLine("" + robot.leftLift.getCurrentPosition());
        telemetry.addLine("" + robot.rightLift.getCurrentPosition());
        //~lift motors will ask for a position to run to
        robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addLine("target position: " + robot.leftLift.getCurrentPosition());
        telemetry.addLine("target position: " + robot.rightLift.getCurrentPosition());
        telemetry.update();

        //~allows the robot to chill out until the play button is pressed
        waitForStart();
        boolean isSpinning = false;
        //~big loop! this is the update loop
        while (opModeIsActive()) {
            //~all these variables in the input section write to doubles based on the controller actions
            boolean aButtonHeld = false;
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x*1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fLeftPower = (y + x + rx) / denominator;
            double bLeftPower = (y - x + rx) / denominator;
            double fRightPower = (y - x - rx) / denominator;
            double bRightPower = (y + x - rx) / denominator;

            //after a bunch of math these powers are assigned to the drivetrain
            robot.fLeftWheel.setPower(fLeftPower * speed);
            robot.bLeftWheel.setPower(bLeftPower * speed);
            robot.fRightWheel.setPower(fRightPower * speed);
            robot.bRightWheel.setPower(bRightPower * speed);

            //~sensitivity limiter
            if(Math.abs(ry)>.1 && Math.abs(y)<.1&&Math.abs(x)<.1&&Math.abs(rx)<.1){
                robot.fRightWheel.setPower(ry);
                robot.bLeftWheel.setPower(ry);
            }
//gamepad1=
            //~checks for controller input on arm position
            if (gamepad2.left_bumper&&leftPosition<0&&rightPosition<0) {
                //~these servos take in a value from 0 to 1, based on the right and left limits
                robot.rArm.setPosition(0); //0 arm out (farthest down)
                robot.lArm.setPosition(0); // 1

                //~sets the target lift positions
                rightPosition += 35;
                leftPosition += 35;

                //~power limiters, 100% power makes the lift shake
                robot.rightLift.setPower(.95);
                robot.leftLift.setPower(.95);

                // If one of them is greater than 0 (out of bound) then set both to zero
                if(rightPosition > 0 || leftPosition > 0){
                    rightPosition = 0;
                    leftPosition = 0;
                }

                // Run to pos
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);

                //~after lift runs, gets the position and writes it (pretty sure this is what aurix is mad about)
                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();
            }
            else if (gamepad2.right_bumper&&leftPosition>-8000&&rightPosition>-8000) {
                robot.rArm.setPosition(0); //0 arm out (farthest down)
                robot.lArm.setPosition(0); // 1
// target: -3750 current: -3850 for max height
                //-1500, target -1680, scoring -2500
                rightPosition -= 30;
                leftPosition -= 30;

                //~same stuff as before, just on the way down - i believe this tries to find the height of the OZ wall
                robot.rightLift.setPower(.95);
                robot.leftLift.setPower(.95);
                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);
                // Run to pos
                robot.rightLift.setTargetPosition(rightPosition);
                robot.leftLift.setTargetPosition(leftPosition);

                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();
                telemetry.addLine("current position: " + a + "," + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();

                //~sabine's solution to the lift freeze issue, do NOT actually implement this unless you prove it works
                /*
                while (robot.rightLift.getCurrentPosition() <= rightPosition || robot.leftLift.getCurrentPosition() <= leftPosition) {
                    a = robot.rightLift.getCurrentPosition();
                    c = robot.leftLift.getCurrentPosition();
                    telemetry.addLine("current position: " + a + "," + c);
                    telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                    telemetry.update();
                }*/
                

            }
//             Teleop Code goes here         }
            //drive climb wrist gamepad1 else is gamepad2

            //~all of this is servo stuff, pretty straightforward
            if (gamepad1.a) {
                robot.claw.setPosition(0); //claw closed
            }
            else if (gamepad1.b){
                robot.claw.setPosition(0.315);
            }

            if (gamepad1.x) {
                robot.claw.setPosition(.415); //
            }

            if (gamepad1.y) {
                // robot.wrist.setPosition(.66); //.73
            }

            if (gamepad1.right_bumper)
            {
                //robot.wrist.setPosition(0);

            }
            else if (gamepad1.left_bumper)
            {
                //robot.wrist.setPosition(1);

            }
            if (gamepad1.dpad_right) //wrist midpoint
            {
                //robot.wrist.setPosition(0.65);
            }

            if (gamepad1.dpad_left) //scoring
            {
                //  robot.wrist.setPosition(.35);
            }
            //add button in
            //~this controls the diff, but it's not fully implemented as of 1330/2/1
            if(-gamepad2.right_stick_y>.1&&rDiff<.51){
                if(pos==2){
                    rDiff = .5;
                    lDiff = .5;
                }
                if(pos==1){
                    rDiff=.36;
                    lDiff=.64;
                }
                rDiff+=.14;
                lDiff-=.14;
                pos++;
                robot.rDiff.setPosition(rDiff);
                robot.lDiff.setPosition(lDiff);
            }
            if(-gamepad2.right_stick_y<.1&&rDiff>.49){
                if(pos==2){
                    rDiff = .5;
                    lDiff = .5;
                }
                if(pos==3){
                    rDiff=.64;
                    lDiff=.36;
                }
                rDiff-=.14;
                lDiff+=.14;
                pos--;
                robot.rDiff.setPosition(rDiff);
                robot.lDiff.setPosition(lDiff);
            }
            if(gamepad2.right_stick_x>.1){
                if(pos==1){
                    rDiff =.267;
                    lDiff = .733;
                }
                if(pos==3){
                    rDiff = .733;
                    lDiff = .267;
                }
                if(pos==2){
                    rDiff = .594;
                    lDiff = .406;
                }
                robot.rDiff.setPosition(rDiff);
                robot.lDiff.setPosition(lDiff);
            }

/*
            else if(gamepad1.dpad_left){ //specimen grab
                robot.wrist.setPosition(.73);
            }*/

          /*  if (gamepad1.dpad_up){ //Wrist Up (when arm is flipped, specimen score, 1 or 0)
                robot.wrist.setPosition(.825);
            }*/

            /*if (gamepad1.dpad_down){ //Wrist Down (when arm is flipped, sample grab, 1 or 0)
                robot.wrist.setPosition(0);
            }*/
//            if(gamepad2.y){
//                telemetry.addLine("Setting slider to zero");
//                telemetry.update();
//                robot.leftLift.setPower(.95);
//                robot.rightLift.setPower(.95);
//                robot.leftLift.setTargetPosition(-2020);
//                robot.rightLift.setTargetPosition(-2020);
//
//                positions = WaitTillTargetReached(50, true);
//                rightPosition = positions[0];
//                leftPosition = positions[1];
//                robot.rightLift.setTargetPosition(-2020);
//                robot.leftLift.setTargetPosition(-2020);
//                rightPosition = positions[0];
//                leftPosition = positions[1];
//                //sleep(750);
//            }




            // TODO: INIT POS 0 FOR HEIGHT
            //~other lift code for presets ig
            if (gamepad2.a){
                telemetry.addLine("Setting slider to zero");
                telemetry.update();
                robot.rArm.setPosition(0); //0 arm out (farthest down)
                robot.lArm.setPosition(0); // 1
                robot.leftLift.setPower(.95);
                robot.rightLift.setPower(.95);
                robot.leftLift.setTargetPosition(0);
                robot.rightLift.setTargetPosition(0);


                positions = WaitTillTargetReached(50, true);
                rightPosition = positions[0];
                leftPosition = positions[1];
                robot.rightLift.setTargetPosition(0);
                robot.leftLift.setTargetPosition(0);
                rightPosition = positions[0];
                leftPosition = positions[1];
            }

//            if (gamepad2.y){
//                //lift
//                robot.leftLift.setTargetPosition(-8000);
//                robot.rightLift.setTargetPosition(-8000);
//
//                positions = WaitTillTargetReached(50, true);
////                robot.rightLift.fsetTargetPosition(rightPosition);
////                robot.leftLift.setTargetPosition(leftPosition);
////                rightPosition = positions[0];
////                leftPosition = positions[1];
//                robot.rArm.setPosition(.9); //arm in 1
//                robot.lArm.setPosition(.1); // 0
//                robot.wrist
//
//                .setPosition(0.5); //wrist midpoint
//            }

            /*if (gamepad2.b)
            {
                robot.rArm.setPosition(1); //arm in 1
                robot.lArm.setPosition(0); // 0

                robot.rightLift.setTargetPosition(-5300);
                robot.leftLift.setTargetPosition(-5300);
                robot.rightLift.setPower(.8);
                robot.leftLift.setPower(.8);
                positions = WaitTillTargetReached(50, true);
//                robot.rightLift.setTargetPosition(rightPosition);
//                robot.leftLift.setTargetPosition(leftPosition);
//                rightPosition = positions[0];
//                leftPosition = positions[1];



                int a = robot.rightLift.getCurrentPosition();
                int c = robot.leftLift.getCurrentPosition();

                telemetry.addLine("Running button command");
                telemetry.addLine("current position: " + a + ", " + c);
                telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
                telemetry.update();

                robot.wrist.setPosition(.825);
            }*/

            //~more lift i guess
            if (gamepad2.x){
                robot.rArm.setPosition(0); //0 arm out (farthest down)
                robot.lArm.setPosition(0); // 1
                robot.rightLift.setPower(.95);
                robot.leftLift.setPower(.95);
                telemetry.addLine("Setting slider to zero");
                telemetry.update();
                robot.leftLift.setTargetPosition(-1680);
                robot.rightLift.setTargetPosition(-1680);
//-1500, target -1680, scoring -2500
                positions = WaitTillTargetReached(50, true);
                rightPosition = positions[0];
                leftPosition = positions[1];
                robot.rightLift.setTargetPosition(-1680);
                robot.leftLift.setTargetPosition(-1680);
                rightPosition = positions[0];
                leftPosition = positions[1];
                //sleep(750);
            }
            if (gamepad1.dpad_up){          //pick up spec from wall
                //robot.wrist.setPosition(0.53);
                //sleep(50);
            }
           /* if (gamepad2.x){
                robot.rArm.setPosition(.8); //arm in 1
                robot.lArm.setPosition(.2); // 0
                telemetry.addLine("Setting slider to zero");
                robot.leftLift.setTargetPosition(0);
                robot.rightLift.setTargetPosition(0);
                robot.wrist.setPosition(.6);
            }*/

            //~even more servo control
            if (gamepad2.dpad_down)
            {robot.rArm.setPosition(0); //0 arm out (farthest down)
                robot.lArm.setPosition(0); // 1
            }
            if (gamepad2.dpad_left)
            {robot.rArm.setPosition(.5);   // init pos
                robot.lArm.setPosition(.5);
            }

            if (gamepad2.dpad_up) //arm out for wall spec grab
            {
                robot.rArm.setPosition(1);
                robot.lArm.setPosition(1);
            }

            if (gamepad2.dpad_right)
            {
                robot.rBar.setPosition(0); // pos slightly above sample
                robot.lBar.setPosition(0); //
            }

            if (gamepad2.dpad_up)
            {robot.lBar.setPosition(0.5);
                robot.rBar.setPosition(0.5);
            }

        }
    }

    //~the actual method for WTTR()
    int[] WaitTillTargetReached(int tolerance, boolean lock){
        int leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
        int rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());
        int check=102930293;
        while(leftDifference > tolerance || rightDifference > tolerance)

        {

            leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
            rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);
            if (check == robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition()) {
                break;
            }
            else {
                check = robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition();
            }
            sleep(1);
            int a = robot.rightLift.getCurrentPosition();
            int c = robot.leftLift.getCurrentPosition();
            telemetry.addLine("current position: " + a + "," + c);
            telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
            telemetry.update();

        }
        int a = robot.rightLift.getCurrentPosition();
        int c = robot.leftLift.getCurrentPosition();
        telemetry.addLine("current position: " + a + "," + c);
        telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
        telemetry.update();
        int[] positions = new int[] {a,c};


        if(!lock)
        {
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);
        }
        return(positions);

    }
    private void cycle() {

    }
}
