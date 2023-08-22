## This repo is here as a storage location as I slowly develop out my personal assistant robot ##
Since I have had back surgery, I have had issues with my back and working with things low to the ground. As a solution, I am turning to a mobility scooter! Now wait, don't turn away yet, this gets better.

I am able to move around just fine, but grabbing things on the ground and pulling heavier items is a challenge. As such, this project's goal is to take an old mobility scooter (ideally a zero turn such as a jazzy), add some intelligence to it via the Robot Operating System and have it a personal assistant to help you carry groceries, do yard work, pull a small trailer, plow a driveway, and anything else that can attach up as an implement.

Goals:
- Keep this opensource. I want anyone to be able to follow along and be able to create the same thing.
- Create a standard, safe, and easy to attach implement system. After all, we don't want to lift anything, that's why we are building this!
- Have it be relatively autonomous. Basic avoidance is a must, and it should be able to follow someone on command.
- Bonuses: Connect to calendars for reminders, meet you outside when you are home from the store, auto homing and recharging.

Currently Equipment:<br>
Radxa Zero (though any Raspberry Pi device that supports ROS2 should work)<br>
Cytron SmartDrive DC Motor Driver (30 Amp) https://www.cytron.io/p-30amp-7v-35v-smartdrive-dc-motor-driver-2-channels<br>
Ryobi 40v Batteries<br>
Buck converters (20 amp, to drop the 40v to ~25v)<br>
