#!/usr/bin/env python3
import re
import sys

def fix_logging_calls(content):
    """Fix all logging calls to use UNLOOK_LOG_* stream macros"""

    # Pattern to match LOG_* calls with format strings
    patterns = [
        (r'LOG_ERROR\("([^"]+)",\s*"([^"]+)"\);', r'UNLOOK_LOG_ERROR("\1") << "\2";'),
        (r'LOG_WARN\("([^"]+)",\s*"([^"]+)"\);', r'UNLOOK_LOG_WARNING("\1") << "\2";'),
        (r'LOG_INFO\("([^"]+)",\s*"([^"]+)"\);', r'UNLOOK_LOG_INFO("\1") << "\2";'),
        (r'LOG_DEBUG\("([^"]+)",\s*"([^"]+)"\);', r'UNLOOK_LOG_DEBUG("\1") << "\2";'),
    ]

    # First pass - simple replacements
    for pattern, replacement in patterns:
        content = re.sub(pattern, replacement, content)

    # Multi-line LOG calls with format strings need special handling
    # Match LOG_* calls that span multiple lines
    multi_patterns = [
        (r'LOG_ERROR\("([^"]+)",\s*\n\s*"([^;]+);', 'LOG_ERROR_MULTI'),
        (r'LOG_WARN\("([^"]+)",\s*\n\s*"([^;]+);', 'LOG_WARN_MULTI'),
        (r'LOG_INFO\("([^"]+)",\s*\n\s*"([^;]+);', 'LOG_INFO_MULTI'),
        (r'LOG_DEBUG\("([^"]+)",\s*\n\s*"([^;]+);', 'LOG_DEBUG_MULTI'),
    ]

    # Complex formatted logging - convert to stream style
    # Pattern: LOG_*(component, format, args...)
    content = re.sub(
        r'LOG_ERROR\("([^"]+)",\s*"([^"]+)",\s*([^)]+)\);',
        lambda m: f'UNLOOK_LOG_ERROR("{m.group(1)}") << "{m.group(2)}" /* TODO: Format args: {m.group(3)} */;',
        content
    )

    content = re.sub(
        r'LOG_WARN\("([^"]+)",\s*"([^"]+)",\s*([^)]+)\);',
        lambda m: f'UNLOOK_LOG_WARNING("{m.group(1)}") << "{m.group(2)}" /* TODO: Format args: {m.group(3)} */;',
        content
    )

    content = re.sub(
        r'LOG_INFO\("([^"]+)",\s*"([^"]+)",\s*([^)]+)\);',
        lambda m: f'UNLOOK_LOG_INFO("{m.group(1)}") << "{m.group(2)}" /* TODO: Format args: {m.group(3)} */;',
        content
    )

    content = re.sub(
        r'LOG_DEBUG\("([^"]+)",\s*"([^"]+)",\s*([^)]+)\);',
        lambda m: f'UNLOOK_LOG_DEBUG("{m.group(1)}") << "{m.group(2)}" /* TODO: Format args: {m.group(3)} */;',
        content
    )

    # Handle multiline format cases
    lines = content.split('\n')
    fixed_lines = []
    i = 0
    while i < len(lines):
        line = lines[i]

        # Check for start of multiline LOG call
        if any(macro in line for macro in ['LOG_ERROR(', 'LOG_WARN(', 'LOG_INFO(', 'LOG_DEBUG(']):
            if ');' not in line:
                # Multiline call - collect all lines
                call_lines = [line]
                i += 1
                while i < len(lines) and ');' not in lines[i]:
                    call_lines.append(lines[i])
                    i += 1
                if i < len(lines):
                    call_lines.append(lines[i])

                # Join and process
                full_call = ' '.join(l.strip() for l in call_lines)

                # Extract component and message
                match = re.match(r'(\s*)LOG_(\w+)\("([^"]+)",\s*"([^"]+)"(?:,\s*([^)]+))?\);', full_call)
                if match:
                    indent = match.group(1) or ''
                    level = match.group(2)
                    component = match.group(3)
                    message = match.group(4)
                    args = match.group(5)

                    level_map = {
                        'ERROR': 'ERROR',
                        'WARN': 'WARNING',
                        'INFO': 'INFO',
                        'DEBUG': 'DEBUG'
                    }

                    if args:
                        # Has format arguments - needs manual fixing
                        fixed = f'{indent}UNLOOK_LOG_{level_map[level]}("{component}")'
                        fixed += f'\n{indent}    << "{message}"; // TODO: Add format args: {args}'
                    else:
                        fixed = f'{indent}UNLOOK_LOG_{level_map[level]}("{component}") << "{message}";'

                    fixed_lines.append(fixed)
                else:
                    # Couldn't parse - keep original
                    fixed_lines.extend(call_lines)
            else:
                fixed_lines.append(line)
        else:
            fixed_lines.append(line)
        i += 1

    return '\n'.join(fixed_lines)

if __name__ == '__main__':
    input_file = '/home/alessandro/unlook-standalone/src/stereo/TemporalStereoProcessor.cpp'

    with open(input_file, 'r') as f:
        content = f.read()

    fixed_content = fix_logging_calls(content)

    with open(input_file, 'w') as f:
        f.write(fixed_content)

    print(f"Fixed logging calls in {input_file}")