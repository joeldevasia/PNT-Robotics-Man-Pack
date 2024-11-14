import wmctrl

# Get the list of windows
list = wmctrl.Window.list()

#Find mapviz window
for window in list:
    if window.wm_class == 'start_screen.py.start_screen.py':
        print(window)