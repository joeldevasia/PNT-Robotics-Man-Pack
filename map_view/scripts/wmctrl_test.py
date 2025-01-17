import wmctrl

# Get the list of windows
list = wmctrl.Window.list()
# print(list)

#Find mapviz window
for window in list:
    if "rviz" in window.wm_class:
        print(window)