#!/usr/bin/env python3

import re

# Read the file
with open('camera_preview_widget.cpp', 'r') as f:
    content = f.read()

# List of all the pointer variables that need null checks
pointers = [
    'sync_status_',
    'left_auto_exposure_button_',
    'right_auto_exposure_button_',
    'left_auto_gain_button_',
    'right_auto_gain_button_',
    'left_exposure_slider_',
    'right_exposure_slider_',
    'left_gain_slider_',
    'right_gain_slider_',
    'fps_slider_',
    'left_camera_label_',
    'right_camera_label_',
    'left_title_label_',
    'right_title_label_',
    'swap_cameras_button_',
    'start_capture_button_',
    'stop_capture_button_',
    'left_camera_status_',
    'right_camera_status_',
    'fps_status_'
]

# For each pointer, add null checks where they're used
for pointer in pointers:
    # Match patterns like: pointer->method()
    pattern = f'({pointer}->\\w+\\([^;]*\\);)'
    replacement = f'if ({pointer}) {{ \\1 }}'
    content = re.sub(pattern, replacement, content)
    
    # Match patterns like: pointer->method(args...)
    pattern = f'({pointer}->\\w+\\([^)]*\\))'
    # Only if it's not already in an if statement
    lines = content.split('\n')
    new_lines = []
    for line in lines:
        if pointer + '->' in line and 'if (' + pointer + ')' not in line:
            # Add null check
            indent = len(line) - len(line.lstrip())
            if '->' in line:
                new_lines.append(' ' * indent + f'if ({pointer}) {{')
                new_lines.append('    ' + line)  # Add extra indent
                new_lines.append(' ' * indent + '}')
            else:
                new_lines.append(line)
        else:
            new_lines.append(line)
    content = '\n'.join(new_lines)

# Write the fixed content
with open('camera_preview_widget.cpp', 'w') as f:
    f.write(content)

print("Fixed null pointer checks in camera_preview_widget.cpp")