# Strelizia
Strelizia! The PROS program named after that robot in that anime. 8301E's code for the Tower Takeover season.

This program makes use of a library I've written called tabu, and it makes testing your robot a breeze! It's an event-oriented way of getting data to and from the microUSB serial. Made even more useful with a wireless USB cable, or a RasPi configured to share its serial ports over bluetooth (that's what we're using, you'll see references to some /dev/rfcomm# in our scripts).

Be on the lookout for another project soon to be open-sourced, tabicat! It's the on-computer counterpart to strelizia. It has a graphical representation of all the things strelizia can be asked to do over serial, and can graph and save data received back from the robot for further analysis. 

## Features
Currently, this project exists mainly to test out control systems under different parameters.

Here's what I'm testing currently:
  - PID (event "pid_test")
    - P, I, D gains
    - kBias
    - useVoltage
  - S Curve (event "simple_follower.test")
    - Displacement, Velocity, Acceleration, and Jerk limits
    - kV, kA
    - stopOnFinish
    - stopBrakeMode
    - feedbackEnabled

Everything is controlled from tabu, rather than in Elliot2 where all configuration and testing is done from the Controller LCD. This is for faster input, better feedback, and most importantly, less cluttered code dealing with UI. There is also nothing on the touchscreen, but it will become Elliot2's auton-switcher when the time comes.

Currently, there is no way for the robot to store any data on the SD card, and there probably won't be a need for it. It is a mostly stateless system, with tests wrapping up in a way that doesn't affect the rest of the program's execution at all. We plan to use PROS' hot/cold linking to make compiling new autonomi fast, and a RasPi wirelessly connected to a PC for wirelessly and safely uploading the autonomi.

## Tabu
Tabu, the amazing event-oriented serial API! It sends two kinds of messages, Events and Replies. An Event is just a plain message, containing JSON data. When an event is sent, it will "wake up" a piece of code relevant to that event, assuming a listener was registered using `tabi_on`. Then, a reply can be sent with feedback to the message. The receiver can use `tabu_on` also to register a reply listener for a particular message. "Big" messages can also be sent, if you're worried your message is too big to be sent as one chunk, it can be split up, waiting for confirmation of reception on each chunk. This is automatically used on `tabu_reply_on` handlers.

Here's an example of it in use:

```
void init_sonic() {
  sonic = std::unique_ptr<pros::ADIUltrasonic>(new pros::ADIUltrasonic('C', 'D'));
  tabu_reply_on("sonic", [&]() -> json {
    return sonic_dist();
  });
}
```
Pretty neat, right? Here, the code sets a listener for the "sonic" event, saying to send the current sonic distance whenever "sonic" is sent across the wire, in just 3 lines.

## Documentation
Currently, this project has no documentation, sorry about that! It's planned for the future, especially for Tabu.

## Earlier Versions
There are none! Unfortunately, strelizia was created with the intention of being a "temporary" project. "Nothing is more permanent than a temporary solution." - said some Russian guy.