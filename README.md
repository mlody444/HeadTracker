# HeadTracker
Re-work of Dennis Frie & Mark Mansur headtracker code for use with a BNO055 Sensor

I've modified the code for the head tracker and wrote a new GUI for it. Based on the BNO055 sensor. This sensor self calibrates once powered up. No need to calibrate connected to the PC.

## Whats needed

1) Arduino Nano - Amazon/Ebay/Many Places.
2) BNO055 Sensor Board - Ebay/Adafruit, https://www.adafruit.com/product/4646, I got mine on ebay, photo below search for BNO055
3) Soldering Iron
4) Short pieces of wire
5) 3.5mm mono stereo cable
6) Small Push Button Switch, for resetting zero

![alt text](https://github.com/dlktdr/HeadTracker/blob/master/Doc/BNO055.jpg?raw=true)

## Assembly

1) With the nano you will get some pin headers. Cut one of these so you have two pins. Solder it into A4(SDA)+A5(SCL), this is the I2C communication pins.
2) Solder two wires one into 3v3 pin and the other in a gnd pin. Leave the other end loose, it's difficult to get at once the board is on.
*** This is a 3.3v IC. Some boards have a voltage regulator on board so you can power them from 5V if desired. Hook up the nano to 5v in this case.
3) The BNO055 has two pins PS0 and PS1. These pins need to be connected to ground to place the chip in I2C mode so the arduino will see it. On my ebay board you make a solder bridge from the S0 to - and from S1 to -. On other brands of boards you may need to connect wires from PS0+PS1 to a ground pin.
4) Solder on the crystal to the back of the board and trim the wires excess wire, I mounted mine flat on the board and used some super glue to hold in place.
5) Solder the board onto the two headers sticking up from the nano. Making sure SDA connects to A4 and SCL connects to A5
6) Cut the two wires sticking up to the proper length and attach them to the 3.3 and gnd pins on the BNO055 board.
7) Cut off one end of your mono 3.5mm cable and strip off the insulation at the end. The outer copper braid connects to a gnd pin. The Center wire connects to D9. This is the cable you will plug into your transmitter to transmit your head orientation
8) Connect the push switch from D11 to GND. This will be used to reset zero when your head is level. I bent one of the pins on the switch and put it into GND, then used a wire to D11 on the other side. 
9) Plug the nano into your computer via USB
10) Program it with the latest code on here plenty of docs on how to do that around. There is a HEX file available in the firmware/bin folder you can program with Xloader( https://www.hobbytronics.co.uk/arduino-xloader) if you don't want to have to download Arduino IDE, add the libraries and compile it yourself.
11) Open the windows app and give it a try!!

###Note: after power up and the sensor calibrates as your moving & the orientation jumps. Zeroing button is required at the moment, also good to have. I haven't really looked into why yet.

![alt text](https://github.com/dlktdr/HeadTracker/blob/master/Doc/Hookup.png?raw=true)

## Note
I was going to make the GUI compatible with the old firmware too but i've made quite a few changes and doubt it will work. If there are many requests I can work on it.

![alt text](https://github.com/dlktdr/HeadTracker/blob/master/ScreenShot.png?raw=true)

## To Do

I've added board orientation choices. Needs work tho, haven't added too many choices. I also have to figure out a better way to do this tho. Everyone isn't going to buy the same board or solder it on in the same orientation.