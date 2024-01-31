#!/bin/sh

for line in `grep -v '\[mainconfig\]' ${CAF_APP_CONFIG_FILE} | sed -e 's/ = /=/' | grep -v '^$'`;
do
        key=$(echo "$line" | cut -d'=' -f1)
        if ! env | grep -q "^$key="; then
                export $line;
	fi
done

python3 /vehicle-obd2.py &

while [ 1 ]
do
        sleep 1
done
