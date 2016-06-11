# mat-PID-hand-tuner
A manual PID tuner written in Matlab. Features a simple GUI.

![Screenshot](http://i.imgur.com/FetNyaz.png "Screenshot")

This little script was created to help solve homework and probably won't have
any practical usability. Still it's a useful utility if you are toying
around with controllers for the first time.

## Usage
Both files (*.m, *.fig) must be placed inside the same directory with their
filename's intact in order for the GUI to launch. To use:
```matlab
% Transfer functions for the controlled system + feedback network
sys = tf([1], [10 1 1]);
fb = tf([1], [80 1]);

% Run UI
pid_hand_tuner(sys, fb)
```

To edit the UI, right click the .fig file inside Matlab's file explorer and select
'Open in GUIDE'. All the logic resides in the .m file.

## License
This project is licensed under the MIT License, available at the LICENSE document of the repository.