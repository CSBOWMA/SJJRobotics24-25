package org.firstinspires.ftc.teamcode;

public class ERROR {
    private static double startCurrent = 0;
    private static double stateTarget = 0;
    private static double minPower = 0.15;
    private static double maxPower = 0.8;

    public static void main(String[] args) {
        startCurrent = 15; //NC
        stateTarget = 55; //C
        for (int i = 15; i < 56; i++) {
            System.out.println("cur: "+i+" power: "+value(15, 10, 2, 8, 2));
            startCurrent++;
        }

        System.out.println();
        System.out.println();
        System.out.println();
        System.out.println();
        startCurrent = 55;
        stateTarget = 15;
        for (int i = 55; i > 14; i--) {
//            valuePositive(55, 10, 2, 8, 2);
            System.out.println("cur: "+i+" power: "+ value(55, 10,2,8,2));

            startCurrent--;

        }


    }
    private static double nextPart(double cur, int startPostion, int endPosition, int startPos2, int endPos2) {

        // Check if out of range
        if (cur > startPostion) {
            return minPower;
        }
        if (cur < endPosition) {
            return endingDamp(cur, startPos2, endPos2);
        }
        // Calculate damping
        int decelDistance = startPostion - endPosition;
        double distanceOnDecel = cur - endPosition;
        double normalizedDistanceOnDecel = distanceOnDecel / decelDistance;
        double flipNormalizedDistanceOnDecel = 1 - normalizedDistanceOnDecel;
        double powerRange = maxPower - minPower;
        double power = powerRange * flipNormalizedDistanceOnDecel + minPower;

        return power;
    }

    private static double endingDamp(double cur, int startPos2, int endpos2) {

        // Check if out of range
        if (cur > startPos2) {
            return maxPower;
        }
        if (cur < endpos2) {
            return minPower;
        }

        // Calculate damping
        int accDistance = startPos2 - endpos2;
        double distanceOnAcc = cur - endpos2;
        double normalizedDistanceOnAcc = distanceOnAcc / accDistance;
        double powerRange = maxPower - minPower;
        double power = powerRange * normalizedDistanceOnAcc + minPower;

        return power;
    }

    private static double getTarget() {
        return stateTarget;
    }

    private static double getCurrent() {
        return startCurrent;
    }


    private  static double value(int startPostionSave, int decelDistance1, int offsetOfStart, int decelDistance2, int offsetOfEnd){
        offsetOfEnd--;
        offsetOfStart--;

        //if target is smaller
        //than going backwards
        if (getTarget()<startPostionSave){
            return nextPart(getCurrent()-getTarget(), startPostionSave-(int)getTarget()-offsetOfStart,
                    startPostionSave-(int)getTarget()-offsetOfStart-decelDistance1, decelDistance2+offsetOfEnd, offsetOfEnd);
        }
        //if target is bigger
        //than going forward
        if (getTarget()>startPostionSave){
            return nextPart(getTarget()-getCurrent(), (int)getTarget()-(startPostionSave+offsetOfStart),
                    (int)getTarget()-(startPostionSave+offsetOfStart+decelDistance1), decelDistance2+offsetOfEnd, offsetOfEnd);
        }
        return 0;
    }
}
