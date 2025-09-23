package frc.robot.openhouse;

public class OpenHouse {

    //An example auto
    public void exampleAuto(){
        for(int index = 0; index < 5; index++){
            robot.turnRight(90);
            robot.moveForward(20);
            robot.turnRight(90);
            robot.moveForward(20);
        }

    }

    public void enterYourOwnNameIdea(){
        robot.moveForward(50);
    }

}