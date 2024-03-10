# KinectMouse

This program controls the mouse by tracking the hand gesture and movement using the Kinect v2 sensor.

## Controls

### The left hand (by default) controls the mouse movement ###

 - raise the hand in the air (closer to the head than the pelvis) to allow control.
 - close the hand **to enter the moving state**.
 - open the hand to exit the moving state.

### The right hand (by default) controls the mouse key ###

 - raise the hand in the air (closer to the head than the pelvis) to allow control.
 - close the hand to press down the **left** mouse button.
 - make the lasso gesture (typically, one finger up is considered lasso) to press down the **right** mouse button.
 - open the hand to release the buttons that were previously pressed.

 > [!NOTE]
 > If the Kinect sensor tracked multiple people, this program will only take account of the first tracked body.
