package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.HashMap;

@Config
public class Transfer {
    private MotionProfile profile;

    private DcMotorEx motor;

    public double manualPower;
    public static double ticksToDeg= 360.0/619.0;
    private double degToTicks= 1.0/ticksToDeg;

    private Servo hinge;

    private RevColorSensorV3 slot1;
    private RevColorSensorV3 slot2;
    private RevColorSensorV3 slot3;

    public static double mult=1;

    public static double kp=0.0085,ki=0.00001,kd=0.0003,kf=0,sp=0.022,sd=00.0007;

    public static double offsetAngle = 0;

    private double spinA = (8.765E-7),spinB=-0.00023861, spinC=0.0157672,spinD=0.702433;

    private  AnalogInput encoder;

    public static double hingeUp =.48,hingeDown = .43;

    private PIDFController controller,Scontroller;

    public static HashMap<String,Double> spinStates;
    public static boolean on =true;
    public static double targetDeg,motionTargetDeg;

    private ElapsedTime timer;

    public static double vMax=3800, aMax =2600;

    private boolean spinning = false;


    private double spinStartTime = 0;
    private double spinDuration = 0;


    private double spinStartPower = 0;
    private double spinEndPower = 0;


    public Transfer(HardwareMap hardwareMap){
        profile = new MotionProfile(vMax, aMax);

        motor=hardwareMap.get(DcMotorEx.class,"spin");
        hinge= hardwareMap.get(Servo.class,"hinge");
        slot1=hardwareMap.get(RevColorSensorV3.class,"slot1");
        slot2=hardwareMap.get(RevColorSensorV3.class,"slot2");
        slot3=hardwareMap.get(RevColorSensorV3.class,"slot3");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = hardwareMap.get(AnalogInput.class, "sp1");


        timer=new ElapsedTime();
        timer.reset();

        controller= new PIDFController(new PIDFCoefficients(kp,ki,kd,kf));
        Scontroller= new PIDFController(new PIDFCoefficients(sp,0,sd,0));
        spinStates = new HashMap<>();
        spinStates.put("SLOT1-COLOR",120D);
        spinStates.put("SLOT2-COLOR",240D);
        spinStates.put("SLOT3-COLOR",0D);

    }

    private double normalizeAngleDeg(double angleDeg) {
        double a = angleDeg % 360.0;
        if (a <= -180.0) a += 360.0;
        if (a > 180.0) a -= 360.0;
        return a;
    }

    public double wrap360(double angleDeg) {
        double a = angleDeg % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }
    public void setTargetDeg(double deg,double nowTime) {
        double newdeg = wrap360(deg);
//        if(newdeg!=targetDeg) {
            profile.startProfile(getPositionDeg(), newdeg, nowTime);
            targetDeg = newdeg;
//        }
    }

    public double getMotionTargetDeg(){return motionTargetDeg;}

    public double getTargetDeg() { return targetDeg; }

    public double getPositionDeg() {
        double ticks = motor.getCurrentPosition();
        double deg = ticks/4096.0*360.0;
//        double deg = ticks*ticksToDeg;

        return wrap360(deg);
//        return deg;
    }

    public double getPositionDegAbs(){

        return ((encoder.getVoltage() / 3.2 * 360 + offsetAngle) % 360);
//        return  (encoder.getVoltage() / 3.2 * 360+offsetAngle) % 360;
//            return encoder.getVoltage();
    }

    public double getPosition() {
        double ticks = motor.getCurrentPosition();
        double deg = ticks;

        /*
        ticks%619/619
         */

//        return wrap360(deg);
        return deg;
    }

    public int getNumBalls(){
        int counter=0;
        for(String i: spinStates.keySet()){
            if(i.contains("PURPLE")||i.contains("GREEN")){
                counter++;
            }
        }
        return counter;
    }





    public void update(double nowTime){
        if (spinning) {
            double t = nowTime - spinStartTime;
            double alpha = Range.clip(t / spinDuration, 0.0, 1.0);


            double power =
                    spinStartPower +
                            alpha * (spinEndPower - spinStartPower);


            motor.setPower(power);


            if (alpha >= 1.0) {
                spinning = false;
            }


            return;
        }
        if (on){
            controller.setCoefficients(new PIDFCoefficients(kp,ki,kd,kf));
            Scontroller.setCoefficients(new PIDFCoefficients(sp,0,sd,0));
            profile.maxAccel= aMax;
            profile.maxVel=vMax;

            double posCmd = profile.getPosition(nowTime);    // desired angle

            motionTargetDeg = posCmd;



            double posErrorDeg = normalizeAngleDeg(posCmd- getPositionDeg());
//            double posErrorTicks = posErrorDeg /360.0*4096;
            controller.updateError(posErrorDeg);
            if(Math.abs(controller.getError())>7) {
                motor.setPower(controller.run());
            }
            else {
                Scontroller.updateError(posErrorDeg);
                motor.setPower(Scontroller.run());
            }
        }
        else{
            motor.setPower(-manualPower);
        }
    }
    //if blank,0 if green 1,if purple 2
    private int getColorID(RevColorSensorV3 sensor){
        double alpha = sensor.alpha();
        double red = (alpha!=0)?sensor.red()/alpha:sensor.red();
        double blue = (alpha!=0)?sensor.blue()/alpha:sensor.blue();
        double green = (alpha!=0)?sensor.green()/alpha:sensor.green();

        /*
          /*
        Purple Ball:
            red:.67-.8
            blue: 1.1-1.5
            green:.75-1.17
        Green Ball:
            red: .3-.62
            blue: .95-1.23
            green: 1.2-1.6
         */
        if (inRange(red,.65,.8)&&inRange(blue,1.1,1.6)&&inRange(green,.77,1.17)){
            return 2;
        }
        else if (inRange(red,.3,.64)&&inRange(blue,.95,1.23)&&inRange(green,1.2,1.7)){
            return 1;

        }

        return 0;


    }

    private boolean inRange(double x,double min,double max){
        return x>=min&&x<=max;
    }





    private String getColorString(int x){
        if (x==1){
            return "GREEN";
        } else if (x==2) {
            return "PURPLE";
        }
        else {
            return "NOTHING";
        }
    }

    public String getMapString(){
        return spinStates.toString();
    }
    public HashMap getMap(){
        return spinStates;
    }
    private int[] numarr= new int[3];

    private double getClosestScan(){
        double error =100000;
        double pos=0;
        if(Math.abs(normalizeAngleDeg(getPositionDeg()-0))<error){
            pos = 0;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-0));
        }

        if (Math.abs(normalizeAngleDeg(getPositionDeg()-120))<error) {
            pos = 120;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-120));
        }

        if(Math.abs(normalizeAngleDeg(getPositionDeg()-240))<error){
            pos = 240;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-240));
        }
        return pos;

    }

    private double getClosestIntake(){
        double error =100000;
        double pos=0;
        if(Math.abs(normalizeAngleDeg(getPositionDeg()-340))<error){
            pos = 340;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-340));
        }

        if (Math.abs(normalizeAngleDeg(getPositionDeg()-220))<error) {
            pos = 220;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-220));
        }

        if(Math.abs(normalizeAngleDeg(getPositionDeg()-100))<error){
            pos = 100;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-100));
        }
        return pos;

    }

    public void spinToScore(double sec) {
        double error =100000;
        double pos=0;
        if(Math.abs(normalizeAngleDeg(getPositionDeg()-275))<error){
            pos = 275;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-275));
        }

        if (Math.abs(normalizeAngleDeg(getPositionDeg()-155))<error) {
            pos = 155;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-155));
        }

        if(Math.abs(normalizeAngleDeg(getPositionDeg()-35))<error){
            pos = 35;
            error = Math.abs(normalizeAngleDeg(getPositionDeg()-35));
        }

        setTargetDeg(pos,sec);
    }


    public double offset=0;
    public void scan(double time){
        double scanPos= getClosestScan();

        setTargetDeg(scanPos,time);

        HashMap<String,Double> temp  =new HashMap<>();
        int slot1Green = getColorID(slot1);
        int slot2Green = getColorID(slot2);
        int slot3Green = getColorID(slot3);
        if(scanPos==0) {
            offset=0;
            temp.put("Slot 1: " + getColorString(slot1Green), 0D);
            temp.put("Slot 2: " + getColorString(slot2Green), 120D);
            temp.put("Slot 3: " + getColorString(slot3Green), 240D);
        } else if(scanPos==120) {
            offset=120;
            temp.put("Slot 1: " + getColorString(slot1Green), 120D);
            temp.put("Slot 2: " + getColorString(slot2Green), 240D);
            temp.put("Slot 3: " + getColorString(slot3Green), 0D);
        } else if(scanPos==240) {
            offset=240;
            temp.put("Slot 1: " + getColorString(slot1Green), 240D);
            temp.put("Slot 2: " + getColorString(slot2Green), 0D);
            temp.put("Slot 3: " + getColorString(slot3Green), 120D);
        }
        numarr[0]=slot1Green;
        numarr[1]=slot2Green;
        numarr[2]=slot3Green;
        spinStates=temp;

    }
    public boolean atTarget(){
        return Math.abs(normalizeAngleDeg(getTargetDeg()-getPositionDeg()))<3;
    }

    public double error(){
        return normalizeAngleDeg(getTargetDeg()-getPositionDeg());
    }


    public String getArrString(){
        return Arrays.toString(numarr);
    }
    public int[] getArr(){
        return numarr;
    }
    public double closer(double org,String key){
        if(spinStates.containsKey(key)&&Math.min(
                Math.abs(spinStates.get(key)-getPositionDeg()),
                Math.abs(spinStates.get(key)-getPositionDeg()+360)
        )<Math.min(
                Math.abs(org-getPositionDeg()),
                Math.abs(org-getPositionDeg()+360)
        )
        ){
            org = spinStates.get(key);
        }
        return org;
    }
    /*
    list = [3]
    double angle;
    for(int i =0;i<3;i++){
        String x = ""';
        for(int j=0;j<3;j++){
            if(map.get(wrap(j*120+i*120)).equals("green"){
                x+="G";
            }
            else if(map.get(wrap(j*120+i*120)).equals("purple"){
                x+="P";
            }
        }
        list[i]=x;
    }



     */

    private String angleToBall(double x){
        if(spinStates.containsKey("Slot 1: GREEN")&&(spinStates.get("Slot 1: GREEN")==x)){
            return "GREEN";
        }
        if(spinStates.containsKey("Slot 1: PURPLE")&&(spinStates.get("Slot 1: PURPLE")==x)){
            return "PURPLE";
        }
        if(spinStates.containsKey("Slot 2: GREEN")&&(spinStates.get("Slot 2: GREEN")==x)){
            return "GREEN";
        }
        if(spinStates.containsKey("Slot 2: PURPLE")&&(spinStates.get("Slot 2: PURPLE")==x)){
            return "PURPLE";
        }
        if(spinStates.containsKey("Slot 3: GREEN")&&(spinStates.get("Slot 3: GREEN")==x)){
            return "GREEN";
        }
        if(spinStates.containsKey("Slot 3: PURPLE")&&(spinStates.get("Slot 3: PURPLE")==x)){
            return "PURPLE";
        }
        return "";
    }


    public String[] getOrderArr(){
        String[] list = new String[3];
        double angle;
        for(int i =0;i<3;i++){
            String x = "";
            for(int j=0;j<3;j++){
                if(angleToBall(wrap360(j*120+i*120)).equals("GREEN")){
                    x+="G";
                }
                else if(angleToBall(wrap360(j*120+i*120)).equals("PURPLE")){
                    x+="P";
                }
            }
            list[i]=x;
        }
        return list;
    }

    private static int scoreMatch(String current, String target) {
        int score = 0;
        int len = Math.min(current.length(), target.length());

        for (int i = 0; i < len; i++) {
            if (current.charAt(i) == target.charAt(i)) {
                score++;
            }
        }
        return score;
    }

    public static int findBestSlot(String[] slots, String target) {
        int bestIndex = 0;
        int bestScore = -1;

        for (int i = 0; i < slots.length; i++) {
            int score = scoreMatch(slots[i], target);

            if (score > bestScore) {
                bestScore = score;
                bestIndex = i;
            }
        }

        return bestIndex;
    }


    public double spin(String x) {



    return wrap360(findBestSlot(getOrderArr(),x)*120-80);//-45
    }



    public void startTransfer(double distance){
        score();
        setManual();

        manualPower= -Range.clip(
                spinA*Math.pow(distance,3)+spinB*Math.pow(distance,2)+spinC*Math.pow(distance,1)+spinD
                ,.6,1);

    }
    public void startTransfer(double pow,boolean custom){
        score();
        setManual();

        manualPower= -pow;

    }
    public void endTransfer(){
//        retract();
        setAuto();
        manualPower=0;
    }

    public void startSpinRamp(double startPower, double endPower, double duration, double nowTime) {
        spinning = true;
        spinStartTime = nowTime;
        spinDuration = duration;
        spinStartPower = startPower;
        spinEndPower = endPower;
    }

    public void endSpinRamp(double nowTime) {
        spinning = false;
        setTargetDeg(getTargetDeg(), nowTime);
        controller.reset();
        Scontroller.reset();
    }



    public void setManual(){
        on=false;
    }
    public void setAuto(){
        on=true;
    }

    //    public void spinTo(double time){
//        double closestBall =10000;
//        String closestBallName="";
//        for (String i:spinStates.keySet()){
//            if (!i.isEmpty()) {
//                if (Math.min(
//                        Math.abs(spinStates.get(i) - getPositionDeg()),
//                        Math.abs(spinStates.get(i) - getPositionDeg() + 360)
//                ) < Math.min(
//                        Math.abs(closestBall - getPositionDeg()),
//                        Math.abs(closestBall - getPositionDeg() + 360)
//                )
//                ) {
//                    closestBall = spinStates.get(i);
//                    closestBallName=i;
//                }
//            }
//        }
//        if(closestBall<1000){
//            setTargetDeg(closestBall,time);
//        }
//    }
    public void removeBall(String x){
        spinStates.remove(x);//temp
    }
    public void score(){
        hinge.setPosition(hingeDown);
    }
    public void retract(){
        hinge.setPosition(hingeUp);
    }

    public boolean isAuto() {
        return on;
    }

    public double getPower() {
        return motor.getPower();
    }


    public class MotionProfile {
        public double maxVel;    // deg/sec
        public double maxAccel;  // deg/sec^2

        private double target;
        private double start;
        private double direction;
        private double distance;

        private double tAccel, tCruise, tDecel, totalTime;
        private double vCruise;

        private double startTime;

        public MotionProfile(double maxVel, double maxAccel) {
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
        }

        public void startProfile(double currentDeg, double targetDeg, double nowTime) {
            start = currentDeg;
            target = targetDeg;

            double error = targetDeg - currentDeg;
            error = normalizeAngleDeg(error);

            direction = Math.signum(error);
            distance = Math.abs(error);

            // Compute motion phases:
            tAccel = maxVel / maxAccel;
            double distAccel = 0.5 * maxAccel * tAccel * tAccel;

            if (2 * distAccel > distance) {
                tAccel = Math.sqrt(distance / maxAccel);
                tCruise = 0;
                tDecel = tAccel;
                vCruise = maxAccel * tAccel;
            } else {
                double distCruise = distance - 2 * distAccel;
                tCruise = distCruise / maxVel;
                tDecel = tAccel;
                vCruise = maxVel;
            }

            totalTime = tAccel + tCruise + tDecel;
            startTime = nowTime;
        }

        public double getPosition(double nowTime) {
            double t = nowTime - startTime;

            if (t < 0) t = 0;
            if (t > totalTime) return target;

            if (t < tAccel) {
                return start + direction * (0.5 * maxAccel * t * t);
            }
            else if (t < tAccel + tCruise) {
                double t2 = t - tAccel;
                return start +
                        direction * (0.5 * maxAccel * tAccel * tAccel + vCruise * t2);
            }
            else {
                double t2 = t - (tAccel + tCruise);
                double distBeforeDecel =
                        0.5 * maxAccel * tAccel * tAccel + vCruise * tCruise;
                double decelDist = vCruise * t2 - 0.5 * maxAccel * t2 * t2;
                return start + direction * (distBeforeDecel + decelDist);
            }
        }

        public double getVelocity(double nowTime) {
            double t = nowTime - startTime;

            if (t < 0) return 0;
            if (t > totalTime) return 0;

            if (t < tAccel)
                return direction * (maxAccel * t);
            else if (t < tAccel + tCruise)
                return direction * vCruise;
            else {
                double t2 = t - (tAccel + tCruise);
                return direction * (vCruise - maxAccel * t2);
            }
        }
    }

}
