package org.firstinspires.ftc.teamcode.Subsystems;

//import needed classes
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This class is used to create a Choice Menu System with Telemetry for Autonomous Programs
 */
public class AutoMenu {
     private Telemetry telemetry;
        private String[] menuItems;
        private int selectionIndex;


        public void init(Telemetry telemetry, String[] menuItems) {
            this.telemetry = telemetry;
            this.menuItems = menuItems;
            selectionIndex = 0;
        }

        public void showMenu() {
            telemetry.clear();
            for(int i = 0; i < menuItems.length; i++){
                if(i == selectionIndex) {
                    telemetry.addLine("> " + menuItems[i]);
                }
            }

            //telemetry.update(); - will update telemetry in AutoBase class
        }

        public int getSelectedOption(){
            return selectionIndex;
        }

        public void navigateUp(){
            selectionIndex = (selectionIndex - 1 + menuItems.length) % menuItems.length;
        }

        public void navigateDown(){
            selectionIndex = (selectionIndex + 1) % menuItems.length;
        }
}
