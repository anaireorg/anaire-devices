#!/bin/bash

dispositivos=( 14745c 4ad440 1474dc 4b9be5 4c2bb1 fcce55 efae33 )

# // Calibrar
for id in "${dispositivos[@]}"; do mosquitto_pub -h portal.anaire.org -p 30183 -q 1 -t config/$id -m '{"update": "ON"}'; done

