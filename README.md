# sol-hackweek-fall-2022
Wall Plotter Robot for Visual Debugging of the Unity Editor

## Step 1: Build Robot
The robot is made using scraps left over from a recent house move and old bits of hobby projects found in a drawer:
- Arduino Uno R3
- 2 x 28BYJ-48 Stepper Motors
- 2 x ULN2003A Unipolar Driver Boards
- Old wire
- Old Network Cable
- Old 5V power supply (phone charger)
- 2 x Cup hooks on shelf blocks
- A piece of old MDF (also used for the pulleys)
- Old kitchen cupboard door as a whiteboard (had no pens though)
- Ikea cardboard boxes and bits of metal that should hold some cabinets to the wall (whoops)
- Sharpie Marker

## Step 2: Write the stepper motor code
This was the bit that took the longest, as the Arduino Uno is a rather slow board, but using a fantastic resource from [ScienceDirect](https://www.sciencedirect.com/topics/engineering/sending-pulse) and a sketch from [iForce2D](http://www.iforce2d.net/sketches/) for stepper motor synchronisation I got it all to work!

## Step 3: Reverse engineer coordinates from line art and the Unity Logo
This was mind numbing, and i did resort to using Excel (booooooooo) but it worked fine.

## Step 4: Next phases
Due to having several projects on currently, I was not able to spend the Hackweek just on hacking, as such what you see in this video is the result of 2-ish days of fun. Next steps are:
1. Write a WebServer for my Raspberry Pi 3b
    1. Ingest an image sent from the Unity Editor Scene or Game Window
    2. Convert this to line art
    3. Calculate G-code to draw image
2. Update Arduino to receive G-code over serial port
3. Make the pen plotter automatically lift
4. Network it!!!
