#!/bin/bash

PACKAGE_NAME="ArduinoFocus_X2.pkg"
BUNDLE_NAME="org.rti-zone.ArduinoFocusX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -s "$app_id_signature" ../build/Release/libArduinoFocus.dylib
fi

mkdir -p ROOT/tmp/ArduinoFocus_X2/
cp "../ArduinoFocus.ui" ROOT/tmp/ArduinoFocus_X2/
cp "../focuserlist ArduinoFocus.txt" ROOT/tmp/ArduinoFocus_X2/
cp "../build/Release/libArduinoFocus.dylib" ROOT/tmp/ArduinoFocus_X2/


if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}

else
    pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
