import wmctrl

# Get the list of windows
list = wmctrl.Window.list()
print(list)
#Find mapviz window
for window in list:
    if window.wm_class == 'manpack-launch-system.py':
        print(window)