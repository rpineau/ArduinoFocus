#!/bin/bash

mkdir -p ROOT/tmp/ArduinoFocus_X2/
cp "../ArduinoFocus.ui" ROOT/tmp/ArduinoFocus_X2/
cp "../focuserlist ArduinoFocus.txt" ROOT/tmp/ArduinoFocus_X2/
cp "../build/Release/libArduinoFocus.dylib" ROOT/tmp/ArduinoFocus_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.ArduinoFocus_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 ArduinoFocus_X2.pkg
pkgutil --check-signature ./ArduinoFocus_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.ArduinoFocus_X2 --scripts Scripts --version 1.0 ArduinoFocus_X2.pkg
fi

rm -rf ROOT
