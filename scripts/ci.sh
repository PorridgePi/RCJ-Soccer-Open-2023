#!/usr/bin/env bash

sections=`grep "^\[" platformio.ini | sort -u | xargs | tr '\[' ' ' | tr '\]' ' ' `
for section in $sections; do
    if [[ "$section" == "env:"* ]]; then
        # echo "Building $SECTION"
        # remove the "env:" prefix
        correct_section=${section:4}
        echo $correct_section
        pio run -e $correct_section
    fi
done
