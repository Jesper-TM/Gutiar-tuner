The projct is a piano tuner that, by an Arduino module can show how wrongly tuned a guitar string is.
The module consists of 3 main components, an electret condenser microphone, LED-display and a LED-ring. When 
a button is preseed the program analyses the singal from the microphone and sets a pre defined target-string,
depending on which string is played on the guitar. A band-pass filter is then applied around the target string
in order to remove unessecary frequencies. Depending on how off-tune the guitar string is, an error is 
calculated. The target sting and error is displayd on the LED-screen. The LED-ring shows the error with light 
indicators which makes the tuner easier to use at a longer distance. 
