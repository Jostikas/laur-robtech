# Translation from linux events, of mouse-relevant values, w/o timestamp (see http://stackoverflow.com/a/16695758)

# Event type: 0: button press, 1: movement, 2: wheel
uint8 type

# Event value: X, Y for movement and wheel, button change and state bitmask for button events
int16 X_btnchange  # Hor movement, Hor scrollwheel, bitmask of whether a button changed
int16 Y_btnstate  # Vert movement, Vert scrollwheel, bitmask of button state.