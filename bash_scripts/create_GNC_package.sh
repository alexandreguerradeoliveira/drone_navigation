#!/bin/bash

packageName=$1
packageName="${packageName,,}"

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
find "${SCRIPTPATH}/.." -type f -exec sed -i -e 's/rocket_/'"$packageName"'_/g' {} \;

rm -rf ${SCRIPTPATH}/../.git

mv ${SCRIPTPATH}/../src/rocket_control.cpp ${SCRIPTPATH}/../src/${packageName}_control.cpp 
mv ${SCRIPTPATH}/../src/rocket_navigation.cpp ${SCRIPTPATH}/../src/${packageName}_navigation.cpp 
mv ${SCRIPTPATH}/../src/rocket_guidance.cpp ${SCRIPTPATH}/../src/${packageName}_guidance.cpp 
mv ${SCRIPTPATH}/../src/rocket_fsm.cpp ${SCRIPTPATH}/../src/${packageName}_fsm.cpp 

mv ${SCRIPTPATH}/../launch/rocket_SIL.launch ${SCRIPTPATH}/../launch/${packageName}_SIL.launch 

mv ${SCRIPTPATH}/../../drone_navigation ${SCRIPTPATH}/../../${packageName}_gnc

echo "Updated name of package to $packageName""_gnc"
