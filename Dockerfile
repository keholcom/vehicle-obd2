FROM alpine:3.15 AS build-stage

RUN	apk update && \
    	apk add python3 py3-pip && \
	pip3 install python-can paho-mqtt pyserial pynmea2 && \
	rm -rf /usr/lib/python3.9/site-packages/pip*

FROM alpine:3.15 AS prod-stage

RUN     apk update && \
        apk add python3 vim can-utils

COPY --from=build-stage /usr/lib/python3.9/site-packages /usr/lib/python3.9/site-packages

COPY startup.sh /startup.sh
RUN chmod 755 /startup.sh

COPY vehicle-obd2.py /vehicle-obd2.py
RUN chmod 755 /vehicle-obd2.py

CMD ". ./startup.sh"
