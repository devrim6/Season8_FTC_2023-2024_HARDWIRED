package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;

public class MeepMeepTesting {
    public static void main(String[] args){
        //MeepMeepStorage robot = new MeepMeepStorage();
        MeepMeepStorage1 robot1 = new MeepMeepStorage1();

        robot1.meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot1.AutoRedStanga3Down)
                .start();

                //.start();
    }
}