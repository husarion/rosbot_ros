#!/bin/bash

# If your ROSbot's IP addr is 10.5.10.64 execute:
# ./sync_with_rosbot.sh 10.5.10.64

sshpass -p "husarion" rsync -vRr --delete ./ husarion@$1:/home/husarion/${PWD##*/}

while inotifywait -r -e modify,create,delete,move ./ ; do
    sshpass -p "husarion" rsync -vRr --delete ./ husarion@$1:/home/husarion/${PWD##*/}
done
