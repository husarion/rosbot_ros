#!/bin/bash

declare -a PEERS=("rosbot" "rviz")
OUTPUT_PATH="./secret"
DDS_FILE_NAME="dds-config.template.xml"

mkdir -p $OUTPUT_PATH
cp $DDS_FILE_NAME $OUTPUT_PATH/dds-config.xml

for (( j=0; j<${#PEERS[@]}; j++ ));
do
    printf "no %d: generating config for \"%s\"\n" $j "${PEERS[$j]}"
    docker run --rm -it husarnet/husarnet:latest husarnet genid > $OUTPUT_PATH/"id_${PEERS[$j]}"
    sed -i "s/${PEERS[$j]}/$(sed -r 's/([a-f0-9:]*)\s.*/\1/g' $OUTPUT_PATH/id_${PEERS[$j]})/g" $OUTPUT_PATH/dds-config.xml
done